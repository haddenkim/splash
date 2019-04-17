#include "ompSolver.h"
#include <omp.h>

using namespace Eigen;

void OmpSolver::advance(System& system, const SimParameters parameters, Stats& stats)
{
	// TODO: find out if this is a no-op when number has not changed, otherwise, need to build some control flow
	// set thread count
	omp_set_num_threads(parameters.numThreads);

	Solver::advance(system, parameters, stats);
}

void OmpSolver::resetGrid(System& system)
{

// reset node
#pragma omp parallel for schedule(static)
	for (int ni = 0; ni < activeNodes_.size(); ni++) {
		// reference to node
		Node& node = *activeNodes_[ni];

		node.mass = 0;
		node.vel.setZero();
		node.force.setZero();

		node.approxParts = 0;
	}

	// reset bookkeeping
	activeNodes_.clear();
}

void OmpSolver::transferP2G(System& system, const SimParameters& parameters)
{
	// auto start = std::chrono::high_resolution_clock::now();

	// compute common inertia-like tensor inverse (D_p)^-1 (paragraph after eq. 176)
	double commonDInvScalar = Interpolation::DInverseScalar(system.dx_);

// pre-compute particle values
#pragma omp parallel for schedule(static)
	for (int pi = 0; pi < system.particles_.size(); pi++) {
		Particle& part = system.particles_[pi];
		// for convenience / optimization, pre-compute part constant contribution to node force VPFT
		part.VPFT = system.constitutiveModel_.computeVolCauchyStress(part.vol0, part.F_E, part.R_E, part.J_P);

		// compute the particle's kernel
		Interpolation& kernel = part.kernel;
		kernel.compute(part.pos, system.dx_);

		// flag nodes as active
		// loop through kernel nodes
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {

					// compute grid frame coordinates of this node
					int gridX = kernel.node0.x() + i;
					int gridY = kernel.node0.y() + j;
					int gridZ = kernel.node0.z() + k;

					// skip if out of bounds
					if (gridX < 0 || gridY < 0 || gridZ < 0
						|| gridX >= system.gridSize_ || gridY >= system.gridSize_ || gridZ >= system.gridSize_) {
						continue;
					}

					// intentionally allowed race condition
					system.nodes_[gridX][gridY][gridZ].approxParts++;
				}
			}
		}
	}

	// auto end	  = std::chrono::high_resolution_clock::now();
	// auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	// start		  = end;
	// printf("after part %li\n", duration.count());

// construct reduced list of active nodes
#pragma omp parallel for schedule(static)
	for (int i = 0; i < system.gridSize_; i++) {
		std::vector<Node*> localActiveNodes;

		for (int j = 0; j < system.gridSize_; j++) {
			for (int k = 0; k < system.gridSize_; k++) {

				Node* node = &system.nodes_[i][j][k];

				if (node->approxParts != 0) {
					localActiveNodes.push_back(node);
				}
			}
		}

		// TODO: investigate a parallel copy_if (aka filter) with reduction
#pragma omp critical
		{
			activeNodes_.insert(activeNodes_.end(), localActiveNodes.begin(), localActiveNodes.end());
		}
	}

	// end		 = std::chrono::high_resolution_clock::now();
	// duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	// start	= end;
	// printf("after list %li\n", duration.count());

#pragma omp parallel for schedule(static)
	for (int ni = 0; ni < activeNodes_.size(); ni++) {
		// reference to node
		Node& node = *activeNodes_[ni];

		// loop through all the nodes in the kernel (aka all neighboring nodes)
		for (int ki = 0; ki < 3; ki++) {
			for (int kj = 0; kj < 3; kj++) {
				for (int kk = 0; kk < 3; kk++) {
					// compute grid frame coordinates of kernel node
					int gridX = node.x + ki - 1;
					int gridY = node.y + kj - 1;
					int gridZ = node.z + kk - 1;

					// skip if out of bounds
					if (gridX < 0 || gridY < 0 || gridZ < 0
						|| gridX >= system.gridSize_ || gridY >= system.gridSize_ || gridZ >= system.gridSize_) {
						continue;
					}

					// reference to kernel node
					const Node& kernelNode = system.nodes_[gridX][gridY][gridZ];

					// loop through particles in kernel node
					for (int npi : kernelNode.ownedParticles) {

						const Particle&		 part   = system.particles_[npi];
						const Interpolation& kernel = part.kernel;

						double   weight			= part.kernel.weight(2 - ki, 2 - kj, 2 - kk);
						Vector3d weightGradient = part.kernel.weightGradient(2 - ki, 2 - kj, 2 - kk);

						// accumulate mass (eq. 172)
						node.mass += weight * part.mass0;

						// compute vector from part to node in world frame (x_i - x_p)
						Vector3d vecPI = kernel.vecPI(2 - ki, 2 - kj, 2 - kk) * system.dx_;

						// accumulate momentum (eq. 173)
						// stores in node velocity, next mpm step computes velocity from momentum
						node.vel += weight * part.mass0 * (part.vel + part.B * commonDInvScalar * vecPI);

						// internal stress force f_i (eq 189)
						node.force -= part.VPFT * weightGradient;
					}
				}
			}
		}
	}

	// end		 = std::chrono::high_resolution_clock::now();
	// duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	// start	= end;
	// printf("after node %li\n", duration.count());
}

void OmpSolver::computeGrid(System& system, const SimParameters& parameters)
{
	// loop through all active nodes
#pragma omp parallel for schedule(static)
	for (int ni = 0; ni < activeNodes_.size(); ni++) {
		// reference to node
		Node& node = *activeNodes_[ni];

		// compute velocity from momentum / mass
		node.vel /= node.mass;

		// TODO: implement other external forces
		// Gravity
		if (parameters.gravityEnabled) {
			node.force.y() -= parameters.gravityG * node.mass;
		}

		// update grid velocities
		node.vel = node.vel + parameters.timestep * node.force / node.mass;

		// enforce boundary conditions
		// node's position  in world space
		Vector3d worldPos = Vector3d(node.x, node.y, node.z) * system.dx_;

		// TODO: implement additional boundary conditions
		// TODO: consider more sophisticated collision response (ex. coefficients of restitution, friction)

		// non-elastic boundaries
		if (worldPos.x() < system.boundary_			  // left wall
			|| worldPos.x() > 1 - system.boundary_	// right wall
			|| worldPos.y() > 1 - system.boundary_	// top wall (ceiling)
			|| worldPos.z() < system.boundary_		  // back wall
			|| worldPos.z() > 1 - system.boundary_) { // front wall
			node.vel.setZero();
		}

		// elastic boundary
		if (worldPos.y() < system.boundary_) { // bottom wall (floor)
			node.vel.y() = std::max(0.0, node.vel.y());
		}
	}
}

void OmpSolver::transferG2P(System& system, const SimParameters& parameters)
{
// loop through part
#pragma omp parallel for schedule(static)
	for (int pi = 0; pi < system.particles_.size(); pi++) {
		Particle& part = system.particles_[pi];

		// reset velocity v_p and affine state B_p and velocity gradient ∇v_p
		part.vel.setZero();
		part.B.setZero();
		part.velGradient.setZero();

		// reference to kernel
		Interpolation& kernel = part.kernel;

		// loop through kernel nodes
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {

					// compute grid frame coordinates of this iter's node
					int gridX = kernel.node0.x() + i;
					int gridY = kernel.node0.y() + j;
					int gridZ = kernel.node0.z() + k;

					// skip if out of bounds
					if (gridX < 0 || gridY < 0 || gridZ < 0
						|| gridX >= system.gridSize_ || gridY >= system.gridSize_ || gridZ >= system.gridSize_) {
						continue;
					}

					double   weight			= kernel.weight(i, j, k);
					Vector3d weightGradient = kernel.weightGradient(i, j, k);

					// reference to node
					Node& node = system.nodes_[gridX][gridY][gridZ];

					// accumulate velocity v_i (eq 175)
					part.vel += weight * node.vel;

					// compute vector from part to node in world frame (x_i - x_p)
					Vector3d vecPI = kernel.vecPI(i, j, k) * system.dx_;

					// acculumate affine state B_i (eq 176)
					part.B += weight * node.vel * vecPI.transpose();

					// accumulate node's deformation update (eq 181)
					part.velGradient += node.vel * weightGradient.transpose();
				}
			}
		}
	}
}

void OmpSolver::computeParticle(System& system, const SimParameters& parameters)
{
#pragma omp parallel for schedule(static)
	for (int pi = 0; pi < system.particles_.size(); pi++) {
		Particle& part = system.particles_[pi];

		// update particle deformation gradient components
		system.constitutiveModel_.updateDeformDecomp(part.F_E, part.R_E, part.F_P, part.J_P, part.velGradient, parameters.timestep);

		// Advection
		part.pos += parameters.timestep * part.vel;

		// enforce boundary conditions
		// TODO: implement additional boundary conditions
		// TODO: consider more sophisticated collision response (ex. coefficients of restitution, friction)

		//
		double farBoundary = 1 - system.boundary_;

		// non-elastic boundaries
		if (part.pos.x() < system.boundary_) // left wall
		{
			part.pos(0) = system.boundary_;
			part.vel(0) = 0;
		}

		if (part.pos.x() > farBoundary) // right wall
		{
			part.pos(0) = farBoundary;
			part.vel(0) = 0;
		}

		if (part.pos.z() < system.boundary_) // back wall
		{
			part.pos(2) = system.boundary_;
			part.vel(2) = 0;
		}

		if (part.pos.z() > farBoundary) // front wall
		{
			part.pos(2) = farBoundary;
			part.vel(2) = 0;
		}

		if (part.pos.y() > farBoundary) // top wall (ceiling)
		{
			part.pos(1) = farBoundary;
			part.vel(1) = 0;
		}

		// update node ownership if needed
		// retrieve current node index
		Eigen::Vector3i currentNodeIndex = part.kernel.node0 + Vector3i::Ones();

		// part position in grid frame
		Eigen::Vector3d partGridPos = part.pos / system.dx_;

		// insert into nearest node
		Eigen::Vector3i newNodeIndex = (partGridPos.array() + 0.5).cast<int>();

		// update if different
		if (newNodeIndex != currentNodeIndex) {
			Node& currentNode = system.nodes_[currentNodeIndex.x()][currentNodeIndex.y()][currentNodeIndex.z()];
			omp_set_lock(&currentNode.lock);
			currentNode.ownedParticles.erase(pi);
			omp_unset_lock(&currentNode.lock);

			// insert into new node
			Node& newNode = system.nodes_[newNodeIndex.x()][newNodeIndex.y()][newNodeIndex.z()];
			omp_set_lock(&newNode.lock);
			newNode.ownedParticles.insert(pi);
			omp_unset_lock(&newNode.lock);
		}
	}
}