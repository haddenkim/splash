#pragma once
#include "solver/solver.h"

class OmpAtomicSolver : public Solver {
public:
	std::string name() override { return "OpenMP Atomic"; };

protected:
	void resetGrid(System& system) override;
	void transferP2G(System& system, const SimParameters& parameters) override;
	void computeGrid(System& system, const SimParameters& parameters) override;
	void transferG2P(System& system, const SimParameters& parameters) override;
	void computeParticle(System& system, const SimParameters& parameters) override;
};