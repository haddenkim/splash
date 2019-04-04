#pragma once

#include "settings/simParameters.h"
#include "settings/stats.h"
#include "state/system.h"
#include <chrono>

class Solver {
public:
	virtual void advance(System& system, const SimParameters parameters, Stats& stats) = 0;

	void clock(int& dest, std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> start)
	{
		auto end		= std::chrono::high_resolution_clock::now();
		auto duration   = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
		dest = duration.count();
	}
};