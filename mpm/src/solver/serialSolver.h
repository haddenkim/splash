#pragma once

#include "solver/solver.h"
#include "state/node.h"

class SerialSolver : public Solver {
public:
	SerialSolver();

	// note: intentionally copying SimParameters to guard against modifications (from GUI thread)
	void simulateOneTick(System& system, const SimParameters parameters) const override;

private:
	/* core mpm steps */
	static void resetGrid(System& system);
	static void computeParticleNodeLinks(System& system);
	static void transferParticleToGrid(System& system, const float timestep);
	static void identifyDegreesOfFreedom(System& system);
	static void computeGridForces(System& system, const SimParameters& parameters);
	static void updateGridVelocities(System& system, const float timestep);
	static void transferGridToParticle(System& system, const float timestep);
	static void advectParticle(System& system, const float timestep);

	/* helpers */
	static std::vector<Node*> getKernelNodes(const Eigen::Vector3d& position, System& system);
};