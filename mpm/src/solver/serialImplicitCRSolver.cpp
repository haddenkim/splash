#include "serialImplicitCRSolver.h"

using namespace Eigen;

void SerialImplicitCRSolver::computeGrid(System& system, const SimParameters& parameters)
{

	// TODO: replace this with more efficient computing on active nodes
	// compute the explicit grid update
	// nodes now store explicit velocity v*
	Solver::computeGrid(system, parameters);

	// reduce DoFs
	computeActiveNodeList(system);

	// setup CR vectors
	// see wiki for notation
	int		 dof = activeNodes_.size() * 3;
	VectorXd CR_x(dof);
	VectorXd CR_r(dof);
	VectorXd CR_p(dof);
	VectorXd CR_Ap(dof);
	VectorXd CR_Ar(dof);

	// use explicit next velocity as initial guess
	for (int ni = 0; ni < activeNodes_.size(); ni++) {
		Node& node				= *activeNodes_[ni];
		CR_x.segment<3>(3 * ni) = node.vel;
	}

	// compute r_0
	computeAx(CR_r, system, CR_x, parameters);
	CR_r = CR_x - CR_r;

	// compute Ar_0
	computeAx(CR_Ar, system, CR_r, parameters);

	// copy for p_0 and Ap_0
	CR_p  = CR_r;
	CR_Ap = CR_Ar;

	// CR loop
	int k;
	for (k = 0; k < parameters.solveMaxIters; k++) {

		// debug
		// printf("step: %i \t total r: %f\n", k, CR_r.norm());

		// printf("x: %f , %f , %f\n", CR_x(0), CR_x(1), CR_x(2));
		// printf("r: %f , %f , %f\n", CR_r(0), CR_r(1), CR_r(2));
		// printf("p: %f , %f , %f\n", CR_p(0), CR_p(1), CR_p(2));
		// printf("Ar: %f , %f , %f\n", CR_Ar(0), CR_Ar(1), CR_Ar(2));
		// printf("Ap: %f , %f , %f\n", CR_Ap(0), CR_Ap(1), CR_Ap(2));
		// printf("\n");

		// for convenience store r^T Ar
		double rTAr_k = CR_r.dot(CR_Ar);

		double alpha = rTAr_k / CR_Ap.dot(CR_Ap); // α_k
		CR_x		 = CR_x + alpha * CR_p;		  // x_k+1
		CR_r		 = CR_r - alpha * CR_Ap;	  // r_k+1

		// log stats
		statsResidual_[k] = CR_r.norm();

		// check tolerance
		if (CR_r.norm() < parameters.solveTolerance) {
			break;
		}

		// compute Ar_k+1
		computeAx(CR_Ar, system, CR_r, parameters);

		double beta = CR_r.dot(CR_Ar) / rTAr_k; // β_k
		CR_p		= CR_r + beta * CR_p;		// p_k+1
		CR_Ap		= CR_Ar + beta * CR_Ap;		// Ap_k+1
	}

	// stats
	statsNumSteps_		= k;
	statsFinalResidual_ = CR_r.norm();

	// printf("final k: %i \tr.norm: %f\n", k, CR_r.norm());

	// debug
	// printf("final \t total r: %f\n", CR_r.norm());

	// printf("x: %f , %f , %f\n", CR_x(0), CR_x(1), CR_x(2));
	// printf("r: %f , %f , %f\n", CR_r(0), CR_r(1), CR_r(2));
	// printf("p: %f , %f , %f\n", CR_p(0), CR_p(1), CR_p(2));
	// printf("Ar: %f , %f , %f\n", CR_Ar(0), CR_Ar(1), CR_Ar(2));
	// printf("Ap: %f , %f , %f\n", CR_Ap(0), CR_Ap(1), CR_Ap(2));
	// printf("\n");
}

void SerialImplicitCRSolver::computeActiveNodeList(System& system)
{
	for (int ni = 0; ni < system.nodeCount(); ni++) {
		Node& node = system.getNode(ni);
		// skip if node has no mass (aka no particles nearby)
		if (node.mass == 0) {
			continue;
		} else {
			activeNodes_.push_back(&node);
		}
	}
}

void SerialImplicitCRSolver::computeAx(Eigen::VectorXd& b, System& system, const Eigen::VectorXd& x, const SimParameters& parameters)
{
	// computes the system's Ax and stores in b

	// for convenience copy x to node
	for (int ni = 0; ni < activeNodes_.size(); ni++) {
		Node& node = *activeNodes_[ni];

		// set δu on each node
		node.differentialU = x.segment<3>(3 * ni);
	}

	// pre compute particle VAFT
	computeParticleVAFTs(system, x);

	// compute b
	double betadTdT = parameters.B * parameters.timestep * parameters.timestep;
	for (int ni = 0; ni < activeNodes_.size(); ni++) {
		Node& node = *activeNodes_[ni];

		b.segment<3>(3 * ni) = x.segment<3>(3 * ni) - betadTdT / node.mass * computeHessianAction(node, system);
	}
}

void SerialImplicitCRSolver::computeParticleVAFTs(System& system, const Eigen::VectorXd& x)
{
	// pre computes V A F^T for every particle (eq 196)

	for (int pi = 0; pi < system.partCount(); pi++) {
		Particle&			 part   = system.getPart(pi);
		const Interpolation& kernel = part.kernel;

		// for every node in kernel
		// accumulate D matrix (eq 197) aka δF (eq 53)
		Matrix3d differentialF_E = Matrix3d::Zero();

		// reset velocity gradient
		part.velGradient.setZero();

		// loop through kernel nodes
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {

					// compute grid frame coordinates of this iter's node
					int gridX = kernel.node0.x() + i;
					int gridY = kernel.node0.y() + j;
					int gridZ = kernel.node0.z() + k;

					// skip if out of bounds
					if (!system.isInBounds(gridX, gridY, gridZ)) {
						continue;
					}

					Vector3d weightGradient = kernel.weightGradient(i, j, k);

					// reference to node
					Node& node = system.getNode(gridX, gridY, gridZ);

					// accumulate ∇v_p for F̂_E (eq 193)
					part.velGradient += node.differentialU * weightGradient.transpose();

					// accumulate D matrix (eq 197)
					differentialF_E += node.differentialU * weightGradient.transpose() * part.F_E;
				}
			}
		}

		// compute  F̂_E (eq 197)
		Eigen::Matrix3d F_E_hat = (Matrix3d::Identity() + part.velGradient) * part.F_E;

		// compute A_p (eq 197) aka δP (eq 56)
		Matrix3d A = part.model->computeFirstPiolaKirchoffDifferential(differentialF_E, F_E_hat, part.J_P);

		// compute V A F^T (eq 196)
		part.VAFT = part.vol0 * A * part.F_E.transpose();
	}
}

Eigen::Vector3d SerialImplicitCRSolver::computeHessianAction(const Node& node, const System& system)
{
	Vector3d hessianAction = Vector3d::Zero();

	// only need to accumulate action of nearby particles
	// loop through all the nodes in the kernel (aka all neighboring nodes)
	for (int ki = 0; ki < 3; ki++) {
		for (int kj = 0; kj < 3; kj++) {
			for (int kk = 0; kk < 3; kk++) {
				// compute grid frame coordinates of kernel node
				int gridX = node.x + ki - 1;
				int gridY = node.y + kj - 1;
				int gridZ = node.z + kk - 1;

				// skip if out of bounds
				if (!system.isInBounds(gridX, gridY, gridZ)) {
					continue;
				}

				// reference to kernel node
				int			ni		   = system.getNodeIndex(gridX, gridY, gridZ);
				const Node& kernelNode = system.getNode(ni);

				// loop through particles in kernel node
				for (Particle* part : kernelNode.ownedParticles) {
					Vector3d weightGradient = part->kernel.weightGradient(2 - ki, 2 - kj, 2 - kk);

					// accumulate hessian action aka differential force δf (eq 196)
					hessianAction -= part->VAFT * weightGradient;
				}
			}
		}
	}

	return hessianAction;
}

Eigen::Matrix3d SerialImplicitCRSolver::computeParticleVAFT(const Particle& part, const Eigen::VectorXd& x)
{
	const Interpolation& kernel = part.kernel;

	// // loop through kernel nodes
	// for (int i = 0; i < 3; i++) {
	// 	for (int j = 0; j < 3; j++) {
	// 		for (int k = 0; k < 3; k++) {

	// 			// compute grid frame coordinates of this iter's node
	// 			int gridX = kernel.node0.x() + i;
	// 			int gridY = kernel.node0.y() + j;
	// 			int gridZ = kernel.node0.z() + k;

	// 			// skip if out of bounds
	// 			if (!system.isInBounds(gridX, gridY, gridZ)) {
	// 				continue;
	// 			}

	// 			Vector3d weightGradient = kernel.weightGradient(i, j, k);

	// 			// reference to node
	// 			Node& node = system.getNode(gridX, gridY, gridZ);

	// 			// accumulate ∇v_p for F̂_E (eq 193)
	// 			part.velGradient += node.differentialU * weightGradient.transpose();

	// 			// accumulate D matrix (eq 197)
	// 			differentialF_E += node.differentialU * weightGradient.transpose() * part.F_E;
	// 		}
	// 	}
	// }

	// // compute  F̂_E (eq 197)
	// Eigen::Matrix3d F_E_hat = (Matrix3d::Identity() + part.velGradient) * part.F_E;

	// // compute A_p (eq 197) aka δP (eq 56)
	// Matrix3d A = part.model->computeFirstPiolaKirchoffDifferential(differentialF_E, F_E_hat, part.J_P);

	// // compute V A F^T (eq 196)
	// part.VAFT = part.vol0 * A * part.F_E.transpose();
}

void SerialImplicitCRSolver::additionalStats(Stats& stats, const System& system)
{
	stats.solveSteps		 = statsNumSteps_;
	stats.solveFinalResidual = statsFinalResidual_;
	for (int i = 0; i < 30; i++) {
		stats.solveResidual[i] = statsResidual_[i];

		statsResidual_[i] = 0;
	}
}