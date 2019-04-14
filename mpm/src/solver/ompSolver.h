#pragma once
#include "solver/solver.h"

class OmpSolver : public Solver {
public:
	void advance(System& system, const SimParameters parameters, Stats& stats) override;

private:
	void resetGrid(System& system);
	void transferP2G(System& system, const SimParameters& parameters);
	void computeGrid(System& system, const SimParameters& parameters);
	void transferG2P(System& system, const SimParameters& parameters);
	void computeParticle(System& system, const SimParameters& parameters);
};