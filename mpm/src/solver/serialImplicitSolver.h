#pragma once
#include "solver/solver.h"

#include <Eigen/Sparse>

class SerialImplicitSolver : public Solver {
public:
	void advance(System& system, const SimParameters parameters, Stats& stats) override;

private:
	void resetGrid(System& system);
	void transferP2G(System& system, const SimParameters& parameters);
	void computeGrid(System& system, const SimParameters& parameters);
	void transferG2P(System& system, const SimParameters& parameters);
	void computeParticle(System& system, const SimParameters& parameters);

	// helpers
	void computeParticleValues(System& system, const SimParameters& parameters);
	void computeEnergyGradient(System& system, const SimParameters& parameters);
	void computeHessian(System& system, const SimParameters& parameters);

	void computeHessianBlock(int i, int j, const Particle& part, const Eigen::Vector3d& wg_ip, const Eigen::Vector3d& wg_jp);

	// time integration bookkeeping
	std::vector<Node*>			 activeNodes_;	 // holds nodes with mass
	std::vector<Eigen::Vector3i> activeNodeIndex_; // ijk index of active node in system grid

	// implicit solve loop data
	Eigen::VectorXd						nodeDisHat_;	 // x̂ − x estimate of next displacement
	Eigen::VectorXd						nodeVelHat_;	 // v̂ estimate of next node velocity
	Eigen::VectorXd						nodeVelDif_;	 // Δv difference of node velocity from solver


	Eigen::VectorXd						energyGradient_; // ∇E
	Eigen::SparseMatrix<double>			hessian_;		 // H
	std::vector<Eigen::Triplet<double>> hessianTriplets_;
};