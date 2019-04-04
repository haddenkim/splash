#include "serialSolver.h"
#include "solver/interpolation.h"
#include <Eigen/Dense>

using namespace Eigen;

void SerialSolver::advance(System& system, const SimParameters parameters, Stats& stats)
{
	resetGrid(system);
	transferP2G(system, parameters);
	computeGrid(system, parameters);
	transferG2P(system, parameters);
	computeParticle(system, parameters);

	stats.simTime += parameters.timestep;
	stats.stepCount ++;
}

void SerialSolver::resetGrid(System& system)
{
	for (int i = 0; i < system.gridSize; i++) {
		for (int j = 0; j < system.gridSize; j++) {
			for (int k = 0; k < system.gridSize; k++) {
				// reference to node
				Node& node = system.nodes_[i][j][k];
				
				node.mass = 0;
				node.vel.setZero();
				node.force.setZero();

				node.particles.clear();
				node.weightGradients.clear();
			}
		}
	}

	// also reset particle bookkeeping values
	for (Particle& part : system.particles_) {
		part.VPFT.setZero();
		part.velGradient.setZero();
	}
}

void SerialSolver::transferP2G(System& system, const SimParameters& parameters)
{
	// compute common inertia-like tensor inverse (D_p)^-1 (paragraph after eq. 176)
	double commonDInvScalar = Interpolation::DInverseScalar(system.dx);

	for (Particle& part : system.particles_) {

		// compute the particle's kernel
		part.kernel = new Interpolation(part.pos, system.dx);

		// loop through kernel nodes
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {

					// compute grid frame coordinates of this iter's node
					int gridX = part.kernel->node0.x() + i;
					int gridY = part.kernel->node0.y() + j;
					int gridZ = part.kernel->node0.z() + k;

					// skip if out of bounds
					if (gridX < 0 || gridY < 0 || gridZ < 0
						|| gridX > system.gridSize || gridY > system.gridSize || gridZ > system.gridSize) {
						continue;
					}

					double   weight			= part.kernel->weight(i, j, k);
					Vector3d weightGradient = part.kernel->weightGradient(i, j, k);

					// reference to node
					Node& node = system.nodes_[gridX][gridY][gridZ];

					// accumulate mass (eq. 172)
					node.mass += weight * part.mass;

					// compute vector from part to node in world frame (x_i - x_p)
					Vector3d vecPI = part.kernel->vecPI(i, j, k) * system.dx;

					// accumulate momentum (eq. 173)
					// stores in node velocity, next mpm step computes velocity from momentum
					node.vel += weight * part.mass * (part.vel + part.B * commonDInvScalar * vecPI);

					// store link to this part for convenience
					node.particles.push_back(&part);

					// store in node
					node.weightGradients.push_back(weightGradient);
				}
			}
		}

		// for convenience / optimization, pre-compute part constant contribution to node force
		// compute current mu and lambda (eq 87)
		double exp	= std::exp(parameters.hardening * (1.0 - part.J_P));
		double mu	 = parameters.mu0 * exp;
		double lambda = parameters.lambda0 * exp;

		// compute determinant of elastic deformation gradient J_E
		double J_E = part.F_E.determinant();

		// compute first Piola-Kirchoff Stresss P (eq 52) * F^T
		Matrix3d PFT = 2 * mu * (part.F_E - part.R_E) * part.F_E.transpose() + (lambda * (J_E - 1) * J_E) * Matrix3d::Identity();

		// compute the per part constant contribution to nodal elastic force V_p * P_p * (F_p)^T (eq 189)
		part.VPFT = part.vol * PFT;
	}
}

void SerialSolver::computeGrid(System& system, const SimParameters& parameters)
{
	// loop through all nodes
	for (int i = 0; i < system.gridSize; i++) {
		for (int j = 0; j < system.gridSize; j++) {
			for (int k = 0; k < system.gridSize; k++) {

				// reference to node
				Node& node = system.nodes_[i][j][k];

				// skip if node has no mass (aka no particles nearby)
				if (node.mass == 0) {
					continue;
				}

				// compute velocity from momentum / mass
				node.vel /= node.mass;

				// internal stress force f_i (eq 189)
				// loop through node's particles to accumulate internal force
				for (int pi = 0; pi < node.particles.size(); pi++) {
					Particle* part = node.particles[pi];

					node.force -= part->VPFT * node.weightGradients[pi];
				}

				// TODO: implement other external forces
				// Gravity
				if (parameters.gravityEnabled) {
					node.force.y() -= parameters.gravityG * node.mass;
				}

				// update grid velocities
				node.vel = node.vel + parameters.timestep * node.force / node.mass;

				// enforce boundary conditions
				// node's position  in world space
				Vector3d worldPos = Vector3d(i, j, k) * system.dx;

				// TODO: implement additional boundary conditions
				// TODO: consider more sophisticated collision response (ex. coefficients of restitution)

				// non-elastic boundaries
				if (worldPos.x() < system.boundary			 // left wall
					|| worldPos.x() > 1 - system.boundary	// right wall
					|| worldPos.y() > 1 - system.boundary	// top wall (ceiling)
					|| worldPos.z() < system.boundary		 // back wall
					|| worldPos.z() > 1 - system.boundary) { // front wall
					node.vel.setZero();
				}

				// elastic boundary
				if (worldPos.y() < system.boundary) { // bottom wall (floor)
					node.vel.y() = std::max(0.0, node.vel.y());
				}
			}
		}
	}
}

void SerialSolver::transferG2P(System& system, const SimParameters& parameters)
{
	// loop through part
	for (Particle& part : system.particles_) {

		// reset velocity v_p and affine state B_p
		part.vel.setZero();
		part.B.setZero();

		// loop through kernel nodes
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {

					// compute grid frame coordinates of this iter's node
					int gridX = part.kernel->node0.x() + i;
					int gridY = part.kernel->node0.y() + j;
					int gridZ = part.kernel->node0.z() + k;

					// skip if out of bounds
					if (gridX < 0 || gridY < 0 || gridZ < 0
						|| gridX > system.gridSize || gridY > system.gridSize || gridZ > system.gridSize) {
						continue;
					}

					double   weight			= part.kernel->weight(i, j, k);
					Vector3d weightGradient = part.kernel->weightGradient(i, j, k);

					// reference to node
					Node& node = system.nodes_[gridX][gridY][gridZ];

					// accumulate velocity v_i (eq 175)
					part.vel += weight * node.vel;

					// compute vector from part to node in world frame (x_i - x_p)
					Vector3d vecPI = part.kernel->vecPI(i, j, k) * system.dx;

					// acculumate affine state B_i (eq 176)
					part.B += weight * node.vel * vecPI.transpose();

					// accumulate node's deformation update (eq 181)
					part.velGradient += node.vel * weightGradient.transpose();
				}
			}
		}
	}
}

void SerialSolver::computeParticle(System& system, const SimParameters& parameters)
{
	for (Particle& part : system.particles_) {
		// Advection
		part.pos += parameters.timestep * part.vel;

		// compute updated elastic deformation gradient F_E_Tilda (eq 181 and paragraph above eq 80)
		Matrix3d F_E_Tilda = (Matrix3d::Identity() + parameters.timestep * part.velGradient) * part.F_E;

		// compute updated deformation gradient F (eq 80)
		Matrix3d F = F_E_Tilda * part.F_P;

		// SVD of F_E_Tilda (eq 83)
		JacobiSVD<Matrix3d> svd(F_E_Tilda, ComputeFullU | ComputeFullV);
		Matrix3d			U   = svd.matrixU();		// U
		Vector3d			Sig = svd.singularValues(); // Σ
		Matrix3d			V   = svd.matrixV();		// V

		// clamp singular values (eq 82)
		for (int i = 0; i < 3; i++) {
			Sig(i) = std::max(Sig(i), 1.0 - parameters.criticalCompression);
			Sig(i) = std::min(Sig(i), 1.0 + parameters.criticalStretch);
		}

		// update part F (eq 84) and R (paragraph under eq 45)
		part.F_E = U * Sig.asDiagonal() * V.transpose();
		part.R_E = U * V.transpose();

		// compute part F_P (eq 86) and its determinant J_P
		part.F_P = part.F_E.inverse() * F;
		part.J_P = part.F_P.determinant();
	}
}