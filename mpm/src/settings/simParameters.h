#pragma once
#include <omp.h>
#include <string>
#include <vector>

struct SimParameters {
	SimParameters()
	{
		// default settings
		selectedSolver = 0;

		timestep = 1e-4;
		numSteps = 1000;

		particlesPerObject = 10000;

		gravityEnabled = true;
		gravityG	   = 200;

		solveTolerance  = 1e-4;
		solveMaxIters   = 5;
		solveStepLength = 1;

		// set max threads
		numThreads   = omp_get_num_procs();
		availThreads = numThreads;
	};

	int	selectedSolver;
	int	numSolvers;
	char** solverNames;

	float timestep;
	int   numSteps;

	int particlesPerObject;

	bool  gravityEnabled;
	float gravityG;

	// implicit solver parameters
	int   solveMaxIters;   //
	float solveTolerance;  // τ
	float solveStepLength; // α

	// OpenMP solver parameters
	int numThreads;
	int availThreads;
};