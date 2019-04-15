#pragma once

struct Stats {
	Stats()
	{
	};

	void reset()
	{
		stepCount = 0;
		simTime   = 0.f;
	}

	int   stepCount;
	float simTime;

	// time of last step
	int timeReset;
	int timeP2G;
	int timeGrid;
	int timeG2P;
	int timePart;

	// total time of step
	int totTimeReset;
	int totTimeP2G;
	int totTimeGrid;
	int totTimeG2P;
	int totTimePart;
};