#pragma once
#include "solver/solver.h"

class SerialImplicitSolver: public Solver{
	 void advance(System& system, const SimParameters parameters, Stats& stats) override;

};