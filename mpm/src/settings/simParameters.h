#pragma once

struct SimParameters {
	SimParameters()
	{
		// default settings
		timestep = 0.001f;

		gravityEnabled = false;
		gravityG	   = 9.8;
	};

	float timestep;

	bool  gravityEnabled;
	float gravityG;
};