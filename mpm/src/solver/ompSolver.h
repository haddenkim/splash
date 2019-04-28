#pragma once
#include "solver/solver.h"

class OmpSolver : public Solver {
public:
	std::string name() override { return "OpenMP Gather"; };

	void advance(System& system, const SimParameters parameters, Stats& stats, int numSteps = 1) override;

protected:
	// virtual void resetGrid(System& system) override ;
	// virtual void transferP2G(System& system, const SimParameters& parameters) override;
	// virtual void computeGrid(System& system, const SimParameters& parameters) override;
	// virtual void transferG2P(System& system, const SimParameters& parameters) override;
	// virtual void computeParticle(System& system, const SimParameters& parameters) override;
};