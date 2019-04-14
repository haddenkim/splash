#pragma once
#include "solver/solver.h"

class OmpSolver : public Solver {
public:
	void advance(System& system, const SimParameters parameters, Stats& stats) override;
};