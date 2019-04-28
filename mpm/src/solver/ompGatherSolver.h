#pragma once
#include "solver/ompSolver.h"

class OmpGatherSolver : public OmpSolver {
public:
	std::string name() override { return "OpenMP Gather"; };

protected:
	void resetGrid(System& system) override;
	void transferP2G(System& system, const SimParameters& parameters) override;
	void computeGrid(System& system, const SimParameters& parameters) override;
	void transferG2P(System& system, const SimParameters& parameters) override;
	void computeParticle(System& system, const SimParameters& parameters) override;

	// helpers
	void p2gPreComputeParts(System& system);
	void p2gSetActiveNodes(System& system);
	void p2gComputeNodes(System& system);
};