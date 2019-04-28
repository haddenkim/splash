#include "ompGatherSolver.h"
#include <omp.h>

using namespace Eigen;

void OmpGatherSolver::resetGrid(System& system)
{

// reset node
#pragma omp parallel for schedule(static)
	for (int ni = 0; ni < activeNodes_.size(); ni++) {
		Node& node = *activeNodes_[ni];

		node.mass = 0;
		node.vel.setZero();
		node.force.setZero();

		node.approxParts = 0;
	}

	// reset bookkeeping
	activeNodes_.clear();
}

void OmpGatherSolver::transferP2G(System& system, const SimParameters& parameters)
{
	// pre-compute particle values
	p2gPreComputeParts(system);

	// construct reduced list of active nodes
	p2gSetActiveNodes(system);

	// transfer to node
	p2gComputeNodes(system);
}

void OmpGatherSolver::p2gPreComputeParts(System& system)
{
// pre-compute particle values
#pragma omp parallel for schedule(static)
	for (int pi = 0; pi < system.partCount(); pi++) {
		Particle& part = system.getPart(pi);
		// for convenience / optimization, pre-compute part constant contribution to node force VPFT
		part.VPFT = part.model->computeVolCauchyStress(part.vol0, part.F_E, part.R_E, part.J_P);

		// compute the particle's kernel
		Interpolation& kernel = part.kernel;
		kernel.compute(part.pos);

		// owning node
		Node& node = system.getNode(part.pos);

		// flag all neighbor nodes as active
		for (Node* kernelNode : node.neighbors) {
			if (kernelNode == nullptr) {
				continue;
			}

			// intentionally allowed race condition
			kernelNode->approxParts++;
		}
	}
}

void OmpGatherSolver::p2gSetActiveNodes(System& system)
{
#pragma omp parallel
	{
		std::vector<Node*> localActiveNodes;

#pragma omp for schedule(static) nowait
		for (int ni = 0; ni < system.nodeCount(); ni++) {
			Node& node = system.getNode(ni);

			if (node.approxParts != 0) {
				localActiveNodes.push_back(&node);
			}
		}

		// TODO: investigate a parallel copy_if (aka filter) with reduction
#pragma omp critical
		{
			activeNodes_.insert(activeNodes_.end(), localActiveNodes.begin(), localActiveNodes.end());
		}
	}
}

void OmpGatherSolver::p2gComputeNodes(System& system)
{

	// compute common inertia-like tensor inverse (D_p)^-1 (paragraph after eq. 176)
	double commonDInvScalar = Interpolation::DInverseScalar();

#pragma omp parallel for schedule(dynamic)
	for (int ni = 0; ni < activeNodes_.size(); ni++) {
		// reference to node
		Node& node = *activeNodes_[ni];

		// loop through all the nodes in the kernel (aka all neighboring nodes)
		for (Node* kernelNode : node.neighbors) {
			if (kernelNode == nullptr) {
				continue;
			}

			// node's position relative to particle's kernel
			int ki = node.x - kernelNode->x + 1;
			int kj = node.y - kernelNode->y + 1;
			int kk = node.z - kernelNode->z + 1;

			// loop through particles in kernel node
			for (Particle* part : kernelNode->ownedParticles) {
				const Interpolation& kernel = part->kernel;

				double   weight			= kernel.weight(ki, kj, kk);
				Vector3d weightGradient = kernel.weightGradient(ki, kj, kk);

				// accumulate mass (eq. 172)
				node.mass += weight * part->mass0;

				// compute vector from part to node in world frame (x_i - x_p)
				Vector3d vecPI = kernel.vecPI(ki, kj, kk);

				// accumulate momentum (eq. 173)
				// stores in node velocity, next mpm step computes velocity from momentum
				node.vel += weight * part->mass0 * (part->vel + part->B * commonDInvScalar * vecPI);

				// internal stress force f_i (eq 189)
				node.force -= part->VPFT * weightGradient;
			}
		}
	}
}

void OmpGatherSolver::computeGrid(System& system, const SimParameters& parameters)
{
	// loop through all active nodes
#pragma omp parallel for schedule(static)
	for (int ni = 0; ni < activeNodes_.size(); ni++) {
		// reference to node
		Node& node = *activeNodes_[ni];

		// compute velocity from momentum / mass
		node.vel /= node.mass;

		// apply external forces
		computeGridExtForces(node, system, parameters);

		// update grid velocities
		node.vel = node.vel + parameters.timestep * node.force / node.mass;

		// process potential collision
		computeGridCollision(node, system, parameters);
	}
}

void OmpGatherSolver::transferG2P(System& system, const SimParameters& parameters)
{
// loop through part
#pragma omp parallel for schedule(static)
	for (int pi = 0; pi < system.partCount(); pi++) {
		Particle& part = system.getPart(pi);

		// reset velocity v_p and affine state B_p and velocity gradient ∇v_p
		part.vel.setZero();
		part.B.setZero();
		part.velGradient.setZero();

		// reference to kernel
		const Interpolation& kernel = part.kernel;

		// owning node
		Node& node = system.getNode(part.pos);

		// flag all neighbor nodes as active
		for (Node* kernelNode : node.neighbors) {
			if (kernelNode == nullptr) {
				continue;
			}

			// kernel node's position relative to particle's kernel
			int i = kernelNode->x - kernel.node0.x();
			int j = kernelNode->y - kernel.node0.y();
			int k = kernelNode->z - kernel.node0.z();

			double   weight			= kernel.weight(i, j, k);
			Vector3d weightGradient = kernel.weightGradient(i, j, k);

			// accumulate velocity v_i (eq 175)
			part.vel += weight * kernelNode->vel;

			// compute vector from part to node in world frame (x_i - x_p)
			Vector3d vecPI = kernel.vecPI(i, j, k);

			// acculumate affine state B_i (eq 176)
			part.B += weight * kernelNode->vel * vecPI.transpose();

			// accumulate node's deformation update (eq 181)
			part.velGradient += kernelNode->vel * weightGradient.transpose();
		}
	}
}

void OmpGatherSolver::computeParticle(System& system, const SimParameters& parameters)
{
#pragma omp parallel for schedule(static)
	for (int pi = 0; pi < system.partCount(); pi++) {
		Particle& part = system.getPart(pi);

		// update particle deformation gradient components
		part.model->updateDeformDecomp(part.F_E, part.R_E, part.F_P, part.J_P, part.velGradient, parameters.timestep);

		// old nearest node
		Node& currentNode = system.getNode(part.pos);

		// Advection
		part.pos += parameters.timestep * part.vel;

		// process potential collision
		computeParticleCollision(part, system, parameters);

		// update node ownership if needed
		// current nearest node
		Node& newNode = system.getNode(part.pos);

		// update if different
		if (&newNode != &currentNode) {
			omp_set_lock(&currentNode.lock);
			currentNode.ownedParticles.erase(&system.getPart(pi));
			omp_unset_lock(&currentNode.lock);

			// insert into new node
			omp_set_lock(&newNode.lock);
			newNode.ownedParticles.insert(&system.getPart(pi));
			omp_unset_lock(&newNode.lock);
		}
	}
}
