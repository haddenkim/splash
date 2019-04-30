#pragma once
#include "settings/constants.h"
#include "solver/solverOmp.h"

class SolverOmpGather : public SolverOmp {
public:
	std::string name() override { return "OpenMP Gather"; };

protected:
	void resetGrid(System& system) override;
	void transferP2G(System& system, const SimParameters& parameters) override;
	void computeGrid(System& system, const SimParameters& parameters) override;
	void transferG2P(System& system, const SimParameters& parameters) override;
	void computeParticle(System& system, const SimParameters& parameters) override;

};