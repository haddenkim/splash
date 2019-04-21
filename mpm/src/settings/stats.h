#pragma once

struct Stats {
	Stats()
	{
		trackEnergy = false;
	};

	void reset()
	{
		stepCount = 0;
		simTime   = 0.f;

		totTimeReset = 0;
		totTimeP2G   = 0;
		totTimeGrid  = 0;
		totTimeG2P   = 0;
		totTimePart  = 0;
	}

	int   stepCount;
	float simTime;

	// time of last step
	unsigned int timeReset;
	unsigned int timeP2G;
	unsigned int timeGrid;
	unsigned int timeG2P;
	unsigned int timePart;

	// total time of step
	unsigned int totTimeReset;
	unsigned int totTimeP2G;
	unsigned int totTimeGrid;
	unsigned int totTimeG2P;
	unsigned int totTimePart;

	// energy stats
	bool  trackEnergy;
	float totalKineticEnergy;
	float totalPotentialEnergy;

	// implicit solver stats
	int   solveSteps;
	float solveFinalResidual;
	float solveResidual[30];
};