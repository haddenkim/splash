#pragma once

#include "settings/simParameters.h"
#include "state/system.h"

class Solver {
public:

	virtual ~Solver()
	{
	}

	// note: copy simParameters to keep thread-safe
	virtual void simulateOneTick(System& system, const SimParameters parameters) const = 0;
};