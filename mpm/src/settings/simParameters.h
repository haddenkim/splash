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

		B				= 1.f;
		solveTolerance  = 1e-4;
		solveMaxIters   = 20;

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
	float B;			   // β interpolation weight
	int   solveMaxIters;   //
	float solveTolerance;  // τ

	// OpenMP solver parameters
	int numThreads;
	int availThreads;
};