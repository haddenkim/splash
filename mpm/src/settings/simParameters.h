#pragma once

struct SimParameters {
	SimParameters()
	{
		// default settings
		solveMethod = SM_EXPLICIT;

		timestep = 1e-4;

		particlesPerObject = 10000;

		gravityEnabled = true;
		gravityG	   = 200;

		solveTolerance = 1e-4;
		solveMaxIters  = 5;
	};

	enum SolveMethod {
		SM_EXPLICIT,
		SM_IMPLICIT,
		SM_OPENMP
	};

	SolveMethod solveMethod;

	float timestep;

	int particlesPerObject;

	bool  gravityEnabled;
	float gravityG;

	// implicit solver parameters
	int	solveMaxIters;   //
	double solveTolerance;  // τ
	double solveStepLength; // α
};