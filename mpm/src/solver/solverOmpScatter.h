#pragma once
#include "settings/constants.h"
#include "solver/solverOmp.h"

class SolverOmpScatter : public SolverOmp {
public:
	std::string name() override { return "OpenMP Scatter"; };

protected:
	void resetGrid(System& system) override;
	void transferP2G(System& system, const SimParameters& parameters) override;
	void computeGrid(System& system, const SimParameters& parameters) override;
	void transferG2P(System& system, const SimParameters& parameters) override;
	void computeParticle(System& system, const SimParameters& parameters) override;

	// helpers
	void p2gSingleBlock(System& system, int bi);

	// particle data

	// node data
};