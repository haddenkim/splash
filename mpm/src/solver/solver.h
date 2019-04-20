#pragma once

#include "settings/simParameters.h"
#include "settings/stats.h"
#include "state/system.h"
#include <chrono>

class Solver {
public:
	virtual std::string name() { return "Default"; };

	virtual void reset();
	virtual void advance(System& system, const SimParameters parameters, Stats& stats);

protected:
	virtual void resetGrid(System& system);
	virtual void transferP2G(System& system, const SimParameters& parameters);
	virtual void computeGrid(System& system, const SimParameters& parameters);
	virtual void transferG2P(System& system, const SimParameters& parameters);
	virtual void computeParticle(System& system, const SimParameters& parameters);

	// helpers
	virtual void computeGridExtForces(Node& node, const System& system, const SimParameters& parameters);
	virtual void computeGridCollision(Node& node, const System& system, const SimParameters& parameters);
	virtual void computeParticleCollision(Particle& part, const System& system, const SimParameters& parameters);

	void clock(unsigned int& current, unsigned int& total, std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds>& start);
};