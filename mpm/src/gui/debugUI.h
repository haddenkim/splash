#pragma once

#include "solver/solver.h"

class DebugUI {
public:
	DebugUI(Solver* solver, System& system, const SimParameters& parameters);

	void draw();

private:
	Solver*				 solver_;
	System&				 system_;
	const SimParameters& parameters_;
};
