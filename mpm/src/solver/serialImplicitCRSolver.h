#pragma once

#include "solver.h"

class SerialImplicitCRSolver : public Solver {
public:
	std::string name() override { return "Implicit CR"; };

private:
	void resetGrid(System& system);
	void computeGrid(System& system, const SimParameters& parameters);

	// helpers
	void updateActiveNodeList(System& system);

	// time integration bookkeeping
	std::vector<Node*>			 activeNodes_;	 // holds nodes with mass
	std::vector<Eigen::Vector3i> activeNodeIndex_; // ijk index of active node in system grid
};
