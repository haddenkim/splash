#include "serialSolver.h"
#include "models/interpolation.h"
#include <Eigen/Dense>

using namespace Eigen;

void SerialSolver::transferP2G(System& system, const SimParameters& parameters)
{
// try storing particles ownership in ALL 27 nodes, and reduce the looping here

	// compute common inertia-like tensor inverse (D_p)^-1 (paragraph after eq. 176)
	double commonDInvScalar = Interpolation::DInverseScalar(system.dx_);

	// pre-compute particle values
	for (Particle& part : system.particles_) {
		// for convenience / optimization, pre-compute part constant contribution to node force VPFT
		part.VPFT = system.constitutiveModel_.computeVolCauchyStress(part.vol0, part.F_E, part.R_E, part.J_P);

		// compute the particle's kernel
		Interpolation& kernel = part.kernel;
		kernel.compute(part.pos, system.dx_);
	}

	// loop through all nodes
	for (int i = 0; i < system.gridSize_; i++) {
		for (int j = 0; j < system.gridSize_; j++) {
			for (int k = 0; k < system.gridSize_; k++) {

				// reference to node
				Node& node = system.nodes_[i][j][k];

				// // compute kernel range
				// Vector3i kStart = Vector3i(i - 1, j - 1, k - 1);
				// Vector3i kRange = Vector3i(3, 3, 3);

				// // adjusts if node is on bounds
				// kRange.array() -= (kStart.array() < 0).cast<int>();
				// kStart.array() += (kStart.array() < 0).cast<int>();
				// int end = system.gridSize_; // for some reason, using gridSize directly in next line fails to compile
				// kRange.array() -= ((kStart + kRange).array() > end).cast<int>();

				// loop through all the nodes in the kernel
				for (int ki = 0; ki < 3; ki++) {
					for (int kj = 0; kj < 3; kj++) {
						for (int kk = 0; kk < 3; kk++) {
							// compute grid frame coordinates of kernel node
							int gridX = i + ki - 1;
							int gridY = j + kj - 1;
							int gridZ = k + kk - 1;

							// skip if out of bounds
							if (gridX < 0 || gridY < 0 || gridZ < 0
								|| gridX >= system.gridSize_ || gridY >= system.gridSize_ || gridZ >= system.gridSize_) {
								continue;
							}

							// reference to node
							const Node& kernelNode = system.nodes_[gridX][gridY][gridZ];

							// loop through particles in kernel node
							for (int pi : kernelNode.ownedParticles) {

								const Particle&		 part   = system.particles_[pi];
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

								// store link to this part for convenience
								node.particles.push_back(pi);

								// store in node
								node.weightGradients.push_back(weightGradient);
							}
						}
					}
				}
			}
		}
	}
}

void SerialSolver::computeParticle(System& system, const SimParameters& parameters)
{
	for (int pi = 0; pi < system.particles_.size(); pi++) {
		Particle& part = system.particles_[pi];

		// update particle deformation gradient components
		system.constitutiveModel_.updateDeformDecomp(part.F_E, part.R_E, part.F_P, part.J_P, part.velGradient, parameters.timestep);

		// Advection
		part.pos += parameters.timestep * part.vel;

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
			currentNode.ownedParticles.erase(pi);

			// insert into new node
			Node& newNode = system.nodes_[newNodeIndex.x()][newNodeIndex.y()][newNodeIndex.z()];
			newNode.ownedParticles.insert(pi);
		}
	}
}