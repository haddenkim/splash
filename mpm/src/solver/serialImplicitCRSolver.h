#pragma once

#include "solver.h"

class SerialImplicitCRSolver : public Solver {
public:
	std::string name() override { return "Implicit CR"; };

private:
	void computeGrid(System& system, const SimParameters& parameters) override;

	// helpers
	void computeActiveNodeList(System& system);

	void			computeAx(Eigen::VectorXd& b, System& system, const Eigen::VectorXd& x, const SimParameters& parameters);
	void			computeParticleVAFTs(System& system, const Eigen::VectorXd& x);
	Eigen::Vector3d computeHessianAction(const Node& node, const System& system);

	//
	Eigen::Matrix3d computeParticleVAFT(const Particle& part, const Eigen::VectorXd& x);

	// analysis
	void additionalStats(Stats& stats, const System& system) override;

	// stats
	int   statsNumSteps_;
	float statsFinalResidual_;
	float statsResidual_[30];
};
