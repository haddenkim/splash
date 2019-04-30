#pragma once

#include <Eigen/Core>
#include <chrono>
#include <string>

class System;
class Node;
class Particle;

class Stats;
class SimParameters;

class Solver {
public:
	virtual std::string name() { return "Base"; };

	virtual void advance(System& system, const SimParameters parameters, Stats& stats, int numSteps = 1);

protected:
	virtual void resetGrid(System& system)										  = 0;
	virtual void transferP2G(System& system, const SimParameters& parameters)	 = 0;
	virtual void computeGrid(System& system, const SimParameters& parameters)	 = 0;
	virtual void transferG2P(System& system, const SimParameters& parameters)	 = 0;
	virtual void computeParticle(System& system, const SimParameters& parameters) = 0;

	// helpers
	virtual void computeGridExtForces(Node& node, const System& system, const SimParameters& parameters);
	virtual void computeGridCollision(Node& node, const System& system, const SimParameters& parameters);
	virtual void computeParticleCollision(Eigen::Vector3d& pos, Eigen::Vector3d& vel, const System& system, const SimParameters& parameters);

	// analysis
	void		 clock(unsigned int& current, unsigned int& total, std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds>& start);
	void		 computeTotalEnergy(Stats& stats, const System& system);
	virtual void additionalStats(Stats& stats, const System& system);
};