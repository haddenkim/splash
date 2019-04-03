#pragma once

#include "settings/simParameters.h"
#include "state/system.h"

class SerialSolver {

public:
	SerialSolver(){};
	~SerialSolver(){};

	static void advance(System& system, const SimParameters parameters);

	// mpm method steps
	static void resetGrid(System& system);

	static void transferP2G(System& system, const SimParameters& parameters);
	static void computeGrid(System& system, const SimParameters& parameters);

	static void transferG2P(System& system, const SimParameters& parameters);
	static void computeParticle(System& system, const SimParameters& parameters);
};
