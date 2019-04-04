#pragma once

#include "settings/simParameters.h"
#include "settings/stats.h"
#include "state/system.h"

class Solver {
public:
	virtual void advance(System& system, const SimParameters parameters, Stats& stats) = 0;
};