#pragma once

#include "settings/simParameters.h"
#include "settings/stats.h"
#include "state/system.h"
#include <chrono>

class Solver {
public:
	virtual void advance(System& system, const SimParameters parameters, Stats& stats) = 0;

	void clock(unsigned int& current, unsigned int& total, std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds>& start);
};