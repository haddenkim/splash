#pragma once

struct SimParameters {
	SimParameters()
	{
		// default settings
		timestep = 1e-4;

		gravityEnabled = true;
		gravityG	   = 200;
	};

	float timestep;

	bool  gravityEnabled;
	float gravityG;



	// implicit solver parameters
	int	solveMaxIters;  //
	double solveTolerance; // τ
	double solveStepLength; // α
};