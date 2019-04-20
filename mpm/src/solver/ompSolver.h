#pragma once
#include "solver/solver.h"

class OmpSolver : public Solver {
public:
	std::string name() override { return "OpenMP"; };

	void advance(System& system, const SimParameters parameters, Stats& stats) override;

protected:
	void resetGrid(System& system) override;
	void transferP2G(System& system, const SimParameters& parameters) override;
	void computeGrid(System& system, const SimParameters& parameters) override;
	void transferG2P(System& system, const SimParameters& parameters) override;
	void computeParticle(System& system, const SimParameters& parameters) override;

	std::vector<Node*> activeNodes_;

};