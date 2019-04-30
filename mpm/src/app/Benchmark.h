#pragma once
#include "settings/simParameters.h"
#include "settings/stats.h"
#include "settings/systemStart.h"
#include "solver/solverType.h"

class System;
class Solver;

class Benchmark {
public:
	Benchmark(SolverType type, SystemStart start, int argc, char* argv[]);

	void parseArgs(int argc, char* argv[]);

	void run(int numSteps);

	void printStats();

private:
	System* system_;
	Solver* solver_;

	SystemStart   start_;
	Stats		  stats_;
	SimParameters simParameters_;
};