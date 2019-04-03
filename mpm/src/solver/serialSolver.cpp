#include "serialSolver.h"
#include "solver/svd.h"
#include <Eigen/Dense>

using namespace Eigen;

void SerialSolver::advance(System& system, const SimParameters parameters)
{
	resetGrid(system);
	transferP2G(system, parameters);
	computeGrid(system, parameters);
	transferG2P(system, parameters);
	computeParticle(system, parameters);
}

void SerialSolver::resetGrid(System& system)
{
	for (int i = 0; i < system.gridSize; i++) {
		for (int j = 0; j < system.gridSize; j++) {
			for (int k = 0; k < system.gridSize; k++) {
				// reference to node
				Node& node = system.nodes_[i][j][k];

				node.vel.setZero();
				node.mass = 0;
				node.force.setZero();

				node.particles.clear();
				node.weightGradients.clear();
			}
		}
	}

	// also reset part bookkeeping values
	for (Particle& part : system.particles_) {
		part.VPFT.setZero();
		part.velocityGradient.setZero();
	}
}

void SerialSolver::transferP2G(System& system, const SimParameters& parameters)
{
	// compute common inertia-like tensor inverse (D_p)^-1 (paragraph after eq. 176)
	double commonDInvScalar = 4.0 / system.dx / system.dx; // (1/4 * (∆x)^2 * I)^-1

	for (Particle& part : system.particles_) {
		// TODO: investigate alternative interpolation kernels
		// for now quadradic kernel (eq 123)

		// part position in grid frame
		Vector3d partGridPos = part.pos.array() / system.dx;

		// lowest node position in grid frame of part's kernel
		Vector3i startNode = (partGridPos.array() - 0.5).cast<int>();

		// vector from base node to part in grid frame
		Vector3d vecIPBase = partGridPos - startNode.cast<double>();

		// compute weight components w_aip (eq 123)
		// Vector3d w[3];
		part.w[0] = 0.50 * (1.5 - vecIPBase.array()).square(); // start node
		part.w[1] = 0.75 - (vecIPBase.array() - 1.0).square(); // middle node
		part.w[2] = 0.50 * (vecIPBase.array() - 0.5).square(); // end node

		// compute weight gradient components ∇w_aip
		// Vector3d wg[3];
		part.wGrad[0] = vecIPBase.array() - 1.5;		  // start node
		part.wGrad[1] = -2.0 * (vecIPBase.array() - 1.0); // middle node
		part.wGrad[2] = vecIPBase.array() - 0.5;		  // end node

		// loop through kernel nodes
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {

					int gridX = startNode.x() + i;
					int gridY = startNode.y() + j;
					int gridZ = startNode.z() + k;

					// skip if out of bounds
					if (gridX < 0 || gridY < 0 || gridZ < 0
						|| gridX > system.gridSize || gridY > system.gridSize || gridZ > system.gridSize) {
						continue;
					}

					// compute weight N_ip (eq. 121)
					double weight = part.w[i].x() * part.w[j].y() * part.w[k].z();

					// compute weight gradient ∇N_ip (eq. after 124)'
					Vector3d weightGradient = Vector3d(part.wGrad[i].x() * part.w[j].y() * part.w[k].z(),
													   part.w[i].x() * part.wGrad[j].y() * part.w[k].z(),
													   part.w[i].x() * part.w[j].y() * part.wGrad[k].z())
						/ system.dx;

					// reference to node
					Node& node = system.nodes_[gridX][gridY][gridZ];

					// accumulate mass (eq. 172)
					node.mass += weight * part.mass;

					// compute vector from part to node in world frame (x_i - x_p)
					Vector3d vecPI = (Vector3d(i, j, k) - vecIPBase) * system.dx;

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
		double exp	= std::exp(parameters.hardening * (1.0 - part.Jp));
		double mu	 = parameters.mu0 * exp;
		double lambda = parameters.lambda0 * exp;

		// compute determinant of elastic deformation gradient J_E
		double Je = part.F.determinant();

		// compute first Piola-Kirchoff Stresss P (eq 52) * F^T
		Matrix3d PFT = 2 * mu * (part.F - part.R) * part.F.transpose() + (lambda * (Je - 1) * Je) * Matrix3d::Identity();

		// compute the per part constant contribution to nodal elastic force V_p * P_p * (F_p)^T (eq 189)
		part.VPFT = part.vol * PFT;
	}
}

void SerialSolver::computeGrid(System& system, const SimParameters& parameters)
{
	// loop through all nodes
	for (int i = 0; i <= system.n; i++) {
		for (int j = 0; j <= system.n; j++) {
			for (int k = 0; k <= system.n; k++) {

				Node& node = system.nodes_[i][j][k];

				// skip if node has no mass (aka no particles nearby)
				if (node.mass > 0) {

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
					// Sticky boundary - left, right, back, front wall, top ceiling
					if (worldPos.x() < system.boundary			 // left wall
						|| worldPos.x() > 1 - system.boundary	// right wall
						|| worldPos.y() > 1 - system.boundary	// top wall (ceiling)
						|| worldPos.z() < system.boundary		 // back wall
						|| worldPos.z() > 1 - system.boundary) { // front wall
						node.vel.setZero();
					}

					// Separate boundary - bottom floor
					if (worldPos.y() < system.boundary) { // bottom wall (floor)
						node.vel.y() = std::max(0.0, node.vel.y());
					}
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

		// TODO: consider storing this during P2G avoiding recompute
		// part position in grid frame
		Vector3d partGridPos = part.pos.array() / system.dx;

		// lowest node position in grid frame of part's kernel
		Vector3i startNode = (partGridPos.array() - 0.5).cast<int>();

		// vector from base node to part in grid frame
		Vector3d vecIPBase = partGridPos - startNode.cast<double>();

		// loop through kernel nodes
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {
					int gridX = startNode.x() + i;
					int gridY = startNode.y() + j;
					int gridZ = startNode.z() + k;

					// skip if out of bounds
					if (gridX < 0 || gridY < 0 || gridZ < 0
						|| gridX > system.gridSize || gridY > system.gridSize || gridZ > system.gridSize) {
						continue;
					}

					// TODO: consider storing this during P2G avoiding recompute
					// compute weight N_ip (eq. 121)
					double weight = part.w[i].x() * part.w[j].y() * part.w[k].z();

					// compute weight gradient ∇N_ip (eq. after 124)'
					Vector3d weightGradient = Vector3d(part.wGrad[i].x() * part.w[j].y() * part.w[k].z(),
													   part.w[i].x() * part.wGrad[j].y() * part.w[k].z(),
													   part.w[i].x() * part.w[j].y() * part.wGrad[k].z())
						/ system.dx;

					// reference to node
					Node& node = system.nodes_[gridX][gridY][gridZ];

					// accumulate velocity v_i (eq 175)
					part.vel += weight * node.vel;

					// compute vector from part to node in world frame (x_i - x_p)
					Vector3d vecPI = (Vector3d(i, j, k) - vecIPBase) * system.dx;

					// acculumate affine state B_i (eq 176)
					part.B += weight * node.vel * vecPI.transpose();

					// accumulate node's deformation update (eq 181)
					part.velocityGradient += node.vel * weightGradient.transpose();
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
		Matrix3d F_E_Tilda = (Matrix3d::Identity() + parameters.timestep * part.velocityGradient) * part.F;

		// compute updated deformation gradient F (eq 80)
		Matrix3d F = F_E_Tilda * part.F_P;

		// Matrix3d U, sig, V;
		// svd(F_E_Tilda, U, sig, V);
		// for (int i = 0; i < 3; i++) {
		// 	sig.row(i)[i] = std::max(sig.row(i)[i], 1.0 - parameters.criticalCompression);
		// 	sig.row(i)[i] = std::min(sig.row(i)[i], 1.0 + parameters.criticalStretch);
		// }

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
		part.F = U * Sig.asDiagonal() * V.transpose();
		part.R = U * V.transpose();

		// compute part F_P (eq 86) and its determinant J_P
		part.F_P = part.F.inverse() * F;
		part.Jp  = part.F_P.determinant();
	}
}