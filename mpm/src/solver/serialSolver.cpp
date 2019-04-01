#include "serialSolver.h"
#include "solver/interpolation.h"
#include <Eigen/Dense>

using namespace Eigen;

SerialSolver::SerialSolver()
	: Solver()
{
}

void SerialSolver::simulateOneTick(System& system, Stats& stats, const SimParameters parameters)
{
	assert(currentStep == SolverStep::SOL_COMPLETE);

	/* Reset grid */
	resetGrid(system);

	/* Particle to grid links, weights and weight gradients */
	computeParticleNodeLinks(system);

	/* Particle to grid transfer (P2G) */
	transferParticleToGrid(system, parameters.timestep);

	/* Identify grid degrees of freedom */
	identifyDegreesOfFreedom(system);

	/* Compute explicit grid forces */
	computeGridForces(system, parameters);

	/* Grid velocity update */
	updateGridVelocities(system, parameters.timestep);

	/* Grid to particle transfer (G2P) + Update particle deformation gradient */
	transferGridToParticle(system, parameters.timestep);

	/* Particle advection */
	advectParticle(system, parameters.timestep);

	// log stats
	stats.simTime += parameters.timestep;
}

void SerialSolver::advanceToStep(SolverStep targetStep, System& system, const SimParameters parameters)
{
	if (targetStep > currentStep || currentStep == SolverStep::SOL_COMPLETE) {

		// progress until target step
		while (currentStep != targetStep) {
			advance(currentStep);

			switch (currentStep) {
			case SolverStep::SOL_RESET:
				resetGrid(system);
				break;

			case SolverStep::SOL_LINKS:
				computeParticleNodeLinks(system);
				break;

			case SolverStep::SOL_P2G:
				transferParticleToGrid(system, parameters.timestep);
				break;

			case SolverStep::SOL_DOF:
				identifyDegreesOfFreedom(system);
				break;

			case SolverStep::SOL_FORCE:
				computeGridForces(system, parameters);
				break;

			case SolverStep::SOL_VEL:
				updateGridVelocities(system, parameters.timestep);
				break;

			case SolverStep::SOL_G2P:
				transferGridToParticle(system, parameters.timestep);
				break;

			case SolverStep::SOL_ADVECT:
				advectParticle(system, parameters.timestep);
				break;
			case SolverStep::SOL_COMPLETE:
				break;

			default:
				break;
			}
		}
	} else {
		// TODO: consider advancing to the next step's target step
		assert(!"Attempting to step backwards.");
	}
}

void SerialSolver::resetGrid(System& system)
{
	// Concurrency note: write system + node + link

	system.activeNodes_ = 0;

	for (Node& node : system.nodes_) {
		node.mass = 0.0;
		node.momentum.setZero();
		node.velocity.setZero();
		node.force.setZero();
		node.active = false;
	}

	// TODO optimize this for performance
	system.linksByNode_.clear();
	system.linksByParticle_.clear();

	for (int ni = 0; ni < system.nodes_.size(); ni++) {
		system.linksByNode_.emplace_back(std::vector<ParticleNodeLink>());
	}

	for (int pi = 0; pi < system.particles_.size(); pi++) {
		system.linksByParticle_.emplace_back(std::vector<ParticleNodeLink>());
	}
}

void SerialSolver::computeParticleNodeLinks(System& system)
{
	// Concurrency note: read particle + node, writes link

	for (int pi = 0; pi < system.particles_.size(); pi++) {
		Particle& particle = system.particles_[pi];

		// retrieve the subset of nodes local to each particle
		auto kernelNodes = getKernelNodes(particle.position, system);

		// loop through subset
		for (Node* node : kernelNodes) {
			assert(node);

			// vector from node to particle in grid frame
			Vector3d vecIP = (particle.position - node->position).cwiseQuotient(system.cellSize_);

			// compute distance components
			double distX = vecIP.x();
			double distY = vecIP.y();
			double distZ = vecIP.z();

			// TODO: consider alternative interpolation kernels (cubic, trilinear)
			// compute weight components w_aip (eq 123)
			double weightX = bSplineQuadratic(distX);
			double weightY = bSplineQuadratic(distY);
			double weightZ = bSplineQuadratic(distZ);

			// compute weight N_ip (eq. 121)
			double weight = weightX * weightY * weightZ;

			// if weight is 0, skip this link
			if (weight == 0) {
				continue;
			}

			// compute weight gradient components ∇w_aip
			double gradX = bSplineQuadraticSlope(distX);
			double gradY = bSplineQuadraticSlope(distY);
			double gradZ = bSplineQuadraticSlope(distZ);

			// compute weight gradient ∇N_ip (eq. after 124)
			Vector3d weightGrad = Vector3d(gradX * weightY * weightZ,
										   weightX * gradY * weightZ,
										   weightX * weightY * gradZ)
									  .cwiseQuotient(system.cellSize_);

			// accumulate inertia-like tensor D_p (eq. 174)
			// Not needed for quadratic or cubic interpolations (paragraph after eq. 176)

			// TODO: find alternative data structure for 2 way (node + particle) iterators
			// perhaps a sparse matrix and interate rows for nodes, columns for particles
			// add to node list
			system.linksByNode_[node->gridIndex].emplace_back(ParticleNodeLink(pi, node->gridIndex, weight, weightGrad));

			// add to particle list
			system.linksByParticle_[pi].push_back(ParticleNodeLink(pi, node->gridIndex, weight, weightGrad));
		}
	}
}

void SerialSolver::identifyDegreesOfFreedom(System& system)
{
	// Concurrency note: write system + node

	// loop through node
	for (int ni = 0; ni < system.nodes_.size(); ni++) {
		Node& node = system.nodes_[ni];

		// node.active = !system.linksByNode_[ni].empty();
		if (node.mass > 0) {
			node.active = true;
			system.activeNodes_++;
		}
	}
}

void SerialSolver::transferParticleToGrid(System& system, const float timestep)
{
	// Concurrency note: read particle + link, write node

	// compute common inertia-like tensor inverse (D_p)^-1 (paragraph after eq. 176)
	Matrix3d commonDInverse = 4.0 * system.cellSize_.cwiseProduct(system.cellSize_).asDiagonal().inverse(); // (1/4 * (∆x)^2 * I)^-1

	// loop through node
	for (int ni = 0; ni < system.nodes_.size(); ni++) {
		Node& node = system.nodes_[ni];

		// skip if node has no particle links
		if (system.linksByNode_[ni].empty()) {
			continue;
		}

		// loop through node's links
		for (const ParticleNodeLink& link : system.linksByNode_[ni]) {
			// convienience/readability
			const Particle& particle = system.particles_[link.particleIndex];

			// accumulate mass (eq. 172)
			node.mass += link.weight * particle.mass;

			// accumulate momentum (eq. 173)
			node.momentum += link.weight * particle.mass * (particle.velocity + particle.affineState * commonDInverse * (node.position - particle.position));
		}

		// compute velocity
		node.velocity = node.momentum / node.mass;
	}
}

void SerialSolver::computeGridForces(System& system, const SimParameters& parameters)
{
	// Concurrency note: read link, writes particle + node

	// compute internal forces
	// Fixed Corotated Constitutive Model (sec 6.3)

	// for convenience / optimization, pre-compute particle constant
	for (Particle& particle : system.particles_) {

		// compute current mu and lambda (eq 87)
		double exp	= std::exp(particle.hardening * (1.0 - particle.plasticDeterminant));
		double mu	 = particle.mu0 * exp;
		double lambda = particle.lambda0 * exp;

		// compute first Piola-Kirchoff Stresss P (eq 52)
		Matrix3d pk1Stress = 2 * mu * (particle.elasticGradient - particle.elasticRotation)
			+ lambda * (particle.elasticDeterminant - 1) * particle.elasticDeterminant * particle.elasticGradient.inverse().transpose();

		// compute the per particle constant contribution to nodal elastic force V_p * P_p * (F_p)^T (eq 189)
		particle.nodalForceContribution = particle.volume * pk1Stress * particle.elasticGradient.transpose();

		assert(!particle.nodalForceContribution.hasNaN());
	}

	// compute node force
	// loop through node
	for (int ni = 0; ni < system.nodes_.size(); ni++) {
		Node& node = system.nodes_[ni];

		// skip inactive nodes
		if (!node.active) {
			continue;
		}

		// loop through node's links
		for (const ParticleNodeLink& link : system.linksByNode_[ni]) {
			// convienience/readability
			const Particle& particle = system.particles_[link.particleIndex];

			// internal stress force f_i (eq 189)
			node.force -= particle.nodalForceContribution * link.weightGradient;
		}

		// TODO: implement external force (ex. gravity)
		if (parameters.gravityEnabled) {
			node.force[1] -= parameters.gravityG * node.mass;
		}

		assert(!node.force.hasNaN());
	}
}

void SerialSolver::updateGridVelocities(System& system, const float timestep)
{
	// Concurrency note: writes node

	for (Node& node : system.nodes_) {
		// skip inactive nodes
		if (!node.active) {
			continue;
		}

		// symplectic euler update (eq 183)
		node.velocity = node.velocity + timestep / node.mass * node.force;

		assert(!node.velocity.hasNaN());

		// TODO: boundary conditions + collisions

		// temp: floor boundary at y = 1
		if (node.position.y() < 1) {
			node.velocity[1] = std::max(node.velocity.y(), 0.0);
		}
	}
}

void SerialSolver::transferGridToParticle(System& system, const float timestep)
{
	// Concurrency note: read node + link, writes particle

	// loop through particle
	for (int pi = 0; pi < system.particles_.size(); pi++) {
		Particle& particle = system.particles_[pi];

		// reset velocity v_p and affine state B_p
		particle.velocity.setZero();
		particle.affineState.setZero();

		// hold accumulation of ∇v_p = Σ_i(v_i * ∇w_ip) (eq 181) (stomakhin step 7)
		Matrix3d velocityGradient = Matrix3d::Zero();

		// loop through particle's links
		for (const ParticleNodeLink& link : system.linksByParticle_[pi]) {
			// convienience/readability
			const Node& node = system.nodes_[link.nodeIndex];

			// accumulate node's deformation update
			velocityGradient += node.velocity * link.weightGradient.transpose();

			// accumulate velocity v_i (eq 175)
			particle.velocity += link.weight * node.velocity;

			assert(!particle.velocity.hasNaN());

			// acculumate affine state B_i (eq 176)
			particle.affineState += link.weight * node.velocity * (node.position - particle.position).transpose();
		}

		// compute updated deformation gradient F_E_Tilda (eq 181 and paragraph above eq 80)
		particle.elasticGradient = (Matrix3d::Identity() + timestep * velocityGradient) * particle.elasticGradient;

		// compute overall deformation gradient F (eq 80)
		Matrix3d deformationGradient = particle.elasticGradient * particle.plasticGradient;

		// SVD decomposition of F_E_Tilda (eq 83)
		JacobiSVD<Matrix3d> svd(particle.elasticGradient, ComputeFullU | ComputeFullV);
		Matrix3d			U	 = svd.matrixU();		  // U
		Vector3d			Sigma = svd.singularValues(); // Σ
		Matrix3d			V	 = svd.matrixV();		  // V

		// clamp singular values (eq 82)
		Sigma = Sigma.cwiseMax(1.0 - particle.criticalCompression).cwiseMin(1.0 + particle.criticalStretch);

		// udpate elastic deformation gradient F_E (eq 84) and its determinant
		particle.elasticGradient	= U * Sigma.asDiagonal() * V.transpose();
		particle.elasticDeterminant = particle.elasticGradient.determinant();

		// udpate plastic deformation gradient F_P (eq 86) and its determinant
		particle.plasticGradient = particle.elasticGradient.inverse() * deformationGradient;
		particle.plasticDeterminant = particle.plasticGradient.determinant();

		// update elastic rotation R_E (paragraph under eq 45)
		particle.elasticRotation = U * V.transpose();
	}

	// TODO: boundary conditions + collisions
}

void SerialSolver::advectParticle(System& system, const float timestep)
{
	// Concurrency note: writes particle

	for (Particle& particle : system.particles_) {
		particle.position += timestep * particle.velocity;
	}
}

std::vector<Node*> SerialSolver::getKernelNodes(const Eigen::Vector3d& position, System& system)
{
	std::vector<Node*> ret;

	// lowest node position in grid frame
	Vector3i startNode = position.cwiseQuotient(system.cellSize_).cast<int>() - system.kernelOffset_;
	// highest node position in grid frame
	Vector3i endNode = startNode + system.kernelSize_;

	// adjust start and end nodes if out of grid bounds
	startNode = startNode.cwiseMax(0.0);
	endNode   = endNode.cwiseMin(system.gridSize_);

	// i,k,j = node position in grid grame
	for (int k = startNode.z(); k < endNode.z(); k++) {
		for (int j = startNode.y(); j < endNode.y(); j++) {
			for (int i = startNode.x(); i < endNode.x(); i++) {
				// retrieve node and add to list
				ret.push_back(system.getNodeAt(i, j, k));
			}
		}
	}

	return ret;
}
