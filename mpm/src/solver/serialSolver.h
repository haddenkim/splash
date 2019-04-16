#pragma once

#include "solver/solver.h"

class SerialSolver : public Solver {
public:
	std::string name() override { return "Node 1st"; };

protected:
	void transferP2G(System& system, const SimParameters& parameters) override;
	void computeParticle(System& system, const SimParameters& parameters) override;
};
