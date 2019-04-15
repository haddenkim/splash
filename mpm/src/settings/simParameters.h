#pragma once

struct SimParameters {
	SimParameters()
	{
		// default settings
		selectedSolver = 0;

		timestep = 1e-4;
		numSteps = 1000;

		particlesPerObject = 100;

		gravityEnabled = true;
		gravityG	   = 200;

		solveTolerance = 1e-4;
		solveMaxIters  = 5;
		solveStepLength = 1;

		numThreads = 4;
	};

	int numSolvers;
	int selectedSolver;
	const char** solverNames;

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