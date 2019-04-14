#pragma once

struct SimParameters {
	SimParameters()
	{
		// default settings
		timestep = 1e-4;

		particlesPerObject = 10000;

		gravityEnabled = true;
		gravityG	   = 200;

		solveImplicit = true;

		solveTolerance = 1e-4;
		solveMaxIters = 5;
	};

	float timestep;

	int particlesPerObject;

	bool  gravityEnabled;
	float gravityG;

	// implicit solver parameters
	bool   solveImplicit;   // false = explicit
	int	solveMaxIters;   //
	double solveTolerance;  // τ
	double solveStepLength; // α
};