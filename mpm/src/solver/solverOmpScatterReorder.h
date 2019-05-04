#pragma once
#include "settings/constants.h"
#include "solver/solverOmpScatter.h"

class SolverOmpScatterReorder : public SolverOmpScatter {
public:
	std::string name() override { return "OpenMP Scatter Reorder"; };

protected:
	void transferP2G(System& system, const SimParameters& parameters) override;
	void transferG2P(System& system, const SimParameters& parameters) override;

	// helpers
	void p2gSingleBlock(System& system, int bi);

	// particle data

	// node data
};