#pragma once

#include "solver/solver.h"

class SerialSolver : public Solver {

public:
	void advance(System& system, const SimParameters parameters, Stats& stats);

	// mpm method steps
private:
	void resetGrid(System& system);

	void transferP2G(System& system, const SimParameters& parameters);
	void computeGrid(System& system, const SimParameters& parameters);

	void transferG2P(System& system, const SimParameters& parameters);
	void computeParticle(System& system, const SimParameters& parameters);
};
