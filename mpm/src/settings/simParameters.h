#pragma once

struct SimParameters {
	SimParameters()
	{
		// default settings
		solveMethod = SM_OPENMP;

		timestep = 1e-4;
		numSteps = 1000;

		particlesPerObject = 10000;

		gravityEnabled = true;
		gravityG	   = 200;

		solveTolerance = 1e-4;
		solveMaxIters  = 5;
		solveStepLength = 1;

		numThreads = 4;
	};

	enum SolveMethod {
		SM_EXPLICIT,
		SM_IMPLICIT,
		SM_OPENMP
	};

	SolveMethod solveMethod;

	float timestep;
	int numSteps;

	int particlesPerObject;

	bool  gravityEnabled;
	float gravityG;

	// implicit solver parameters
	int	solveMaxIters;   //
	float solveTolerance;  // τ
	float solveStepLength; // α

	// OpenMP solver parameters
	int numThreads;

};