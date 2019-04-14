#include "serialImplicitSolver.h"

using namespace Eigen;

void SerialImplicitSolver::advance(System& system, const SimParameters parameters, Stats& stats)
{
	auto start = std::chrono::high_resolution_clock::now();

	resetGrid(system);
	clock(stats.timeReset, start);

	transferP2G(system, parameters);
	clock(stats.timeP2G, start);

	computeGrid(system, parameters);
	clock(stats.timeGrid, start);

	transferG2P(system, parameters);
	clock(stats.timeG2P, start);

	computeParticle(system, parameters);
	clock(stats.timePart, start);

	// log stats
	stats.simTime += parameters.timestep;
	stats.stepCount++;
}

void SerialImplicitSolver::resetGrid(System& system)
{
	// reset bookkeeping
	activeNodes_.clear();
	activeNodeIndex_.clear();

	// reset node
	for (int i = 0; i < system.gridSize_; i++) {
		for (int j = 0; j < system.gridSize_; j++) {
			for (int k = 0; k < system.gridSize_; k++) {
				Node& node = system.nodes_[i][j][k];

				node.mass = 0;
				node.vel.setZero();
				node.force.setZero();

				node.particles.clear();
				node.weightGradients.clear();

				node.activeNodeIndex = -1;
			}
		}
	}

	// reset particle bookkeeping values
	for (Particle& part : system.particles_) {
		part.VPFT.setZero();
		part.velGradient.setZero();

		part.F_E_hat.setZero();
		part.VdP_dF.setZero();
	}
}

void SerialImplicitSolver::transferP2G(System& system, const SimParameters& parameters)
{
	// compute common inertia-like tensor inverse (D_p)^-1 (paragraph after eq. 176)
	double commonDInvScalar = Interpolation::DInverseScalar(system.dx_);

	for (Particle& part : system.particles_) {

		// for convenience / optimization, pre-compute part constant contribution to node force VPFT
		part.VPFT = system.constitutiveModel_.computeVolCauchyStress(part.vol0, part.F_E, part.R_E, part.J_P);

		// compute the particle's kernel
		Interpolation& kernel = part.kernel;
		kernel.compute(part.pos, system.dx_);

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
						|| gridX > system.gridSize_ || gridY > system.gridSize_ || gridZ > system.gridSize_) {
						continue;
					}

					double   weight			= kernel.weight(i, j, k);
					Vector3d weightGradient = kernel.weightGradient(i, j, k);

					// reference to node
					Node& node = system.nodes_[gridX][gridY][gridZ];

					// accumulate mass (eq. 172)
					node.mass += weight * part.mass0;

					// compute vector from part to node in world frame (x_i - x_p)
					Vector3d vecPI = kernel.vecPI(i, j, k) * system.dx_;

					// accumulate momentum (eq. 173)
					// stores in node velocity, next mpm step computes velocity from momentum
					node.vel += weight * part.mass0 * (part.vel + part.B * commonDInvScalar * vecPI);

					// store link to this part for convenience
					node.particles.push_back(&part);

					// store in node
					node.weightGradients.push_back(weightGradient);
				}
			}
		}
	}
}

void SerialImplicitSolver::computeGrid(System& system, const SimParameters& parameters)
{
	// produce list of active nodes for reduced DoF solves
	for (int i = 0; i < system.gridSize_; i++) {
		for (int j = 0; j < system.gridSize_; j++) {
			for (int k = 0; k < system.gridSize_; k++) {

				// reference to node
				Node& node = system.nodes_[i][j][k];

				// skip if node has no mass (aka no particles nearby)
				if (node.mass == 0) {
					continue;
				}

				// compute velocity from momentum / mass
				node.vel /= node.mass;

				// update node record
				node.activeNodeIndex = activeNodes_.size();

				// add to active node list
				activeNodes_.push_back(&node);
				activeNodeIndex_.emplace_back(Vector3i(i, j, k));
			}
		}
	}

	// resize implicit solve data
	int dof = activeNodes_.size() * 3; // 3 dimension
	nodeDisHat_.resize(dof);
	nodeVelHat_.resize(dof);
	nodeVelDif_.resize(dof);

	energyGradient_.resize(dof);
	hessian_.resize(dof, dof);
	hessianTriplets_.clear();

	// fill next node velocities with initial guess
	for (int ni = 0; ni < activeNodes_.size(); ni++) {
		// initial guess = current node velocity
		nodeVelHat_.segment<3>(ni * 3) = activeNodes_[ni]->vel;
	}

	// implicit solver loop
	int si; // si - solver iteration
	for (si = 0; si < parameters.solveMaxIters; si++) {
		// for convenience/optimization pre-compute the estimated node displacements
		// x̂_i - x_i = ∆tv_i (paragraph above eq 192)
		nodeDisHat_ = nodeVelHat_ * parameters.timestep;

		// compute system's energy gradient
		// **root finding (eq 200)
		computeParticleValues(system, parameters);
		computeEnergyGradient(system, parameters);

		// check against tolerance
		printf("energyGradient norm: %0.3f\n", energyGradient_.norm());
		if (energyGradient_.norm() < parameters.solveTolerance) {
			break;
		}

		// ** root finding ∂h/∂v (eq 201)
		computeHessian(system, parameters);

		// ** root finding adjustments (eq 200)
		// above computes ∂f/∂v
		SparseMatrix<double> mass(dof,dof);
		mass.setIdentity();
		hessian_ = mass + parameters.timestep * parameters.timestep * hessian_;

		// solve for delta
		SparseQR<SparseMatrix<double>, COLAMDOrdering<int>> solver;
		solver.compute(hessian_);
		nodeVelDif_ = solver.solve(-energyGradient_);

		// update for next iteration
		nodeVelHat_ += nodeVelDif_;
		energyGradient_.setZero();
		hessian_.setZero();
		hessianTriplets_.clear();
	}

	printf("solved in %i steps\n", si);

	// update node data
	for (int ani = 0; ani < activeNodes_.size(); ani++) {
		Node& node = *activeNodes_[ani];

		node.vel = nodeVelHat_.segment<3>(3 * ani);
	}
}

void SerialImplicitSolver::transferG2P(System& system, const SimParameters& parameters)
{
	// loop through part
	for (Particle& part : system.particles_) {

		// reset velocity v_p and affine state B_p
		part.vel.setZero();
		part.B.setZero();

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
						|| gridX > system.gridSize_ || gridY > system.gridSize_ || gridZ > system.gridSize_) {
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

void SerialImplicitSolver::computeParticle(System& system, const SimParameters& parameters)
{
	for (Particle& part : system.particles_) {
		// update particle deformation gradient components
		system.constitutiveModel_.updateDeformDecomp(part.F_E, part.R_E, part.F_P, part.J_P, part.velGradient, parameters.timestep);

		// Advection
		part.pos += parameters.timestep * part.vel;
	}
}

void SerialImplicitSolver::computeParticleValues(System& system, const SimParameters& parameters)
{
	// compute estimates for velGradient, VPFT, dP_dF
	for (Particle& part : system.particles_) {

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
						|| gridX > system.gridSize_ || gridY > system.gridSize_ || gridZ > system.gridSize_) {
						continue;
					}

					// retrieve this node's weight gradient ∇w_ip
					Vector3d weightGradient = kernel.weightGradient(i, j, k);

					// index of this node in the active node list
					Node& node = system.nodes_[gridX][gridY][gridZ];
					int   ani  = node.activeNodeIndex;

					// accumulate node's deformation update (eq 193)
					part.velGradient += nodeDisHat_.segment<3>(ani * 3) * weightGradient.transpose();
				}
			}
		}

		// copy deformation gradient components so that current timestep's components remains unchanged for next implicit loop iter
		Matrix3d F_E_hat = part.F_E; // F̂_E
		Matrix3d R_E_hat = part.R_E; // R̂_E
		Matrix3d F_P_hat = part.F_P; // F̂_P
		double   J_P_hat = part.J_P; // Ĵ_P

		// compute all particles elastic deformation gradient estimate F̂_E
		system.constitutiveModel_.updateDeformDecomp(F_E_hat, R_E_hat, F_P_hat, J_P_hat, part.velGradient, parameters.timestep);

		// compute first Piola-Kirchoff Stress estimate P̂ (eq 52)
		Matrix3d P_hat = system.constitutiveModel_.computeFirstPiolaKirchoff(F_E_hat, R_E_hat, J_P_hat);

		// compute the per part constant contribution to nodal elastic force estimate V_p * P̂_p * (F_p)^T (eq 194)
		part.VPFT = part.vol0 * P_hat * part.F_E.transpose();

		// compute the particle's ∂P/∂F (for eq 198)
		part.VdP_dF = part.vol0 * system.constitutiveModel_.computeFirstPiolaKirchoffDerivative(F_E_hat, R_E_hat, J_P_hat);
	}
}

void SerialImplicitSolver::computeEnergyGradient(System& system, const SimParameters& parameters)
{
	// loop through active nodes, building Energy Gradient ∇E
	for (int ani = 0; ani < activeNodes_.size(); ani++) {
		// reference to node
		Node& nodeI = *activeNodes_[ani];

		for (int pi = 0; pi < nodeI.particles.size(); pi++) {
			Particle& part = *nodeI.particles[pi];

			// internal stress force estimate f̂_i (eq 194)
			// loop through node's particles to accumulate internal force
			nodeI.force -= part.VPFT * nodeI.weightGradients[pi];
		}

		// TODO: implement other external forces
		// Gravity
		if (parameters.gravityEnabled) {
			nodeI.force.y() -= parameters.gravityG * nodeI.mass;
		}

		// enter node's energy gradient (eq 205 derivative)
		// ** root finding (eq 200)
		energyGradient_.segment<3>(ani * 3) = nodeI.mass * (nodeVelHat_.segment<3>(ani * 3) - nodeI.vel) - parameters.timestep * nodeI.force;

		assert(!energyGradient_.segment<3>(ani * 3).hasNaN());
	}
}

void SerialImplicitSolver::computeHessian(System& system, const SimParameters& parameters)
{
	// TODO: determine most efficient loop order
	// loop through particles vs loop through nodes

	// compute Hessian (eq 198)
	for (const Particle& part : system.particles_) {

		// reference to kernel
		const Interpolation& kernel = part.kernel;

		// loop through kernel nodes as I
		for (int i_i = 0; i_i < 3; i_i++) {
			for (int j_i = 0; j_i < 3; j_i++) {
				for (int k_i = 0; k_i < 3; k_i++) {

					// compute grid frame coordinates of this iter's node
					int gridX = kernel.node0.x() + i_i;
					int gridY = kernel.node0.y() + j_i;
					int gridZ = kernel.node0.z() + k_i;

					// skip if out of bounds
					if (gridX < 0 || gridY < 0 || gridZ < 0
						|| gridX > system.gridSize_ || gridY > system.gridSize_ || gridZ > system.gridSize_) {
						continue;
					}

					// retrieve node I's weight gradient ∇w_jp
					Vector3d weightGradient_ip = kernel.weightGradient(i_i, j_i, k_i);

					// index of this node in the active node list
					Node& nodeI = system.nodes_[gridX][gridY][gridZ];
					int   ani   = nodeI.activeNodeIndex;

					// loop through kernel nodes as J
					for (int i_j = 0; i_j < 3; i_j++) {
						for (int j_j = 0; j_j < 3; j_j++) {
							for (int k_j = 0; k_j < 3; k_j++) {
								// reuse gridX,Y,Z
								gridX = kernel.node0.x() + i_i;
								gridY = kernel.node0.y() + j_i;
								gridZ = kernel.node0.z() + k_i;

								// skip if out of bounds
								if (gridX < 0 || gridY < 0 || gridZ < 0
									|| gridX > system.gridSize_ || gridY > system.gridSize_ || gridZ > system.gridSize_) {
									continue;

									// retrieve node I's weight gradient ∇w_jp
									Vector3d weightGradient_jp = kernel.weightGradient(i_j, j_j, k_j);

									// index of this node in the active node list
									Node& nodeJ = system.nodes_[gridX][gridY][gridZ];
									int   anj   = nodeJ.activeNodeIndex;

									// compute this particle-nodeI-nodeJ triplet's hessian parts
									computeHessianBlock(ani, anj, part, weightGradient_ip, weightGradient_jp);
								}
							}
						}
					}
				}
			}
		}
	}

	// construct hessian matrix
	hessian_.setFromTriplets(hessianTriplets_.begin(), hessianTriplets_.end());
}

void SerialImplicitSolver::computeHessianBlock(int i, int j, const Particle& part, const Eigen::Vector3d& wg_ip, const Eigen::Vector3d& wg_jp)
{

	// i = ani, α = xyz
	// j = bnj, τ = xyz
	// compute 1 particle-nodeI-nodeJ triplet's partial sum for the Hessian (eq 198)

	// TODO compress this into MMMs and MVMs
	for (int a = 0; a < 3; a++) {	 // α
		for (int t = 0; t < 3; t++) { // τ
			double H_iajt = 0;

			for (int b = 0; b < 3; b++) {			  // β
				for (int s = 0; s < 3; s++) {		  // σ
					for (int w = 0; w < 3; w++) {	 // ω
						for (int c = 0; c < 3; c++) { // γ

							// retrieve ∂2 Ψ/∂F_αβ ∂F_τσ (para above 74)
							double VdP_dF_abts = part.VdP_dF.coeff(3 * a + b, 3 * t + s);

							H_iajt += VdP_dF_abts * wg_jp.coeff(w) * wg_ip.coeff(c) * part.F_E_hat.coeff(w, s) * part.F_E_hat.coeff(c, b);
						}
					}
				}
			}

			int row = 3 * i + a;
			int col = 3 * j + t;
			hessianTriplets_.emplace_back(Triplet<double>(row, col, H_iajt));
		}
	}
}
