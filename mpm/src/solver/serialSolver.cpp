#include "serialSolver.h"
#include "solver/interpolation.h"
#include <Eigen/Dense>

using namespace Eigen;

SerialSolver::SerialSolver()
	: Solver()
{
}

void SerialSolver::simulateOneTick(System& system, const SimParameters parameters) const
{
	/* Reset grid */
	resetGrid(system);

	/* Particle to grid links, weights and weight gradients */
	computeParticleNodeLinks(system);

	/* Particle to grid transfer (P2G) */
	transferParticleToGrid(system, parameters.timestep);

	/* Identify grid degrees of freedom */
	identifyDegreesOfFreedom(system);

	/* Compute explicit grid forces */
	// computeGridForces(system, parameters);

	/* Grid velocity update */
	updateGridVelocities(system, parameters.timestep);

	/* Grid to particle transfer (G2P) + Update particle deformation gradient */
	transferGridToParticle(system, parameters.timestep);

	/* Particle advection */
	advectParticle(system, parameters.timestep);
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
		system.linksByParticle_.emplace_back(std::vector<ParticleNodeLink*>());
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

			// compute weight gradient components ∇w_aip
			double gradX = bSplineQuadraticSlope(distX);
			double gradY = bSplineQuadraticSlope(distY);
			double gradZ = bSplineQuadraticSlope(distZ);

			// compute weight N_ip (eq. 121)
			double weight = weightX * weightY * weightZ;

			// if weight is 0, skip this link
			if (weight == 0) {
				continue;
			}

			// compute weight gradient ∇N_ip(eq. after 124)
			Vector3d weightGrad = Vector3d(gradX * weightY * weightZ,
										   weightX * gradY * weightZ,
										   weightX * weightY * gradZ)
									  .cwiseQuotient(system.cellSize_);

			// accumulate inertia-like tensor D_p (eq. 174)
			// Not needed for quadratic or cubic interpolations (paragraph after eq. 176)

			// store on node list
			int li = system.linksByNode_[node->gridIndex].size();
			system.linksByNode_[node->gridIndex].emplace_back(ParticleNodeLink(&particle, node, weight, weightGrad));

			// add particle to particle list
			ParticleNodeLink* link = &system.linksByNode_[node->gridIndex].at(li);
			system.linksByParticle_[pi].push_back(link);
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
			const Particle& particle = *link.particle;

			// accumulate mass (eq. 172)
			node.mass += link.weight * particle.mass;

			// accumulate momentum (eq. 173)
			node.momentum += link.weight * particle.mass * (particle.velocity + particle.affineState * commonDInverse * (node.position - particle.position));
		}

		// compute velocity
		node.velocity = node.momentum / node.mass;

		assert(node.velocity.x() > 0);
	}
}

void SerialSolver::computeGridForces(System& system, const SimParameters& parameters)
{
	// Concurrency note: read link, writes particle + node

	// for convenience / optimization, pre-compute particle constant
	for (Particle& particle : system.particles_) {
		// compute deformation gradient inverse transpose F^(-T)
		Matrix3d defGradInvTran = particle.deformationGradient.inverse().transpose();

		// compute determinent of deformation gradient J
		double J = particle.deformationGradient.determinant();

		// compute first Piola-Kirchoff Stresss P (eq 48)
		Matrix3d pk1Stress = particle.mu * (particle.deformationGradient - defGradInvTran) + particle.lambda * log(J) * defGradInvTran;

		// compute the per particle constant contribution to nodal elastic force V_p * P_p * (F_p)^T (eq 189)
		particle.nodalForceContribution = particle.volume * pk1Stress * particle.deformationGradient.transpose();
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
			const Particle& particle = *link.particle;

			// internal stress force f_i (eq 189)
			node.force -= particle.nodalForceContribution * link.weightGradient;
		}

		// TODO: implement external force (ex. gravity)
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
		node.velocity = node.velocity + timestep * node.force / node.mass;

		assert(!node.velocity.hasNaN());
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

		// TODO: find name for this
		// holds Σ_i (eq 181)
		Matrix3d deformUpdate = Matrix3d::Zero();

		// loop through particle's links
		for (ParticleNodeLink* link : system.linksByParticle_[pi]) {
			// convienience/readability
			const Node& node = *link->node;

			// accumulate node's deformation update
			deformUpdate += node.velocity * link->weightGradient.transpose();

			// accumulate velocity v_i (eq 175)
			particle.velocity += link->weight * node.velocity;

			assert(!particle.velocity.hasNaN());

			// acculumate affine state B_i (eq 176)
			particle.affineState += link->weight * node.velocity * (node.position - particle.position).transpose();
		}

		// update deformation gradient F_p (eq 181)
		particle.deformationGradient = (Matrix3d::Identity() + timestep * deformUpdate) * particle.deformationGradient;
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
