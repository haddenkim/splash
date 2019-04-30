#include "solver/solver.h"
#include "settings/simParameters.h"
#include "settings/stats.h"
#include "state/system.h"
#include <Eigen/Dense>

using namespace Eigen;

void Solver::advance(System& system, const SimParameters parameters, Stats& stats, int numSteps)
{
	for (int t = 0; t < numSteps; t++) {
		auto start = std::chrono::high_resolution_clock::now();

		resetGrid(system);
		clock(stats.timeReset, stats.totTimeReset, start);

		transferP2G(system, parameters);
		clock(stats.timeP2G, stats.totTimeP2G, start);

		computeGrid(system, parameters);
		clock(stats.timeGrid, stats.totTimeGrid, start);

		transferG2P(system, parameters);
		clock(stats.timeG2P, stats.totTimeG2P, start);

		computeParticle(system, parameters);
		clock(stats.timePart, stats.totTimePart, start);

		// log stats
		stats.simTime += parameters.timestep;
		stats.stepCount++;

		if (stats.trackEnergy) {
			computeTotalEnergy(stats, system);
		}

		additionalStats(stats, system);
	}
}

void Solver::computeGridExtForces(Node& node, const System& system, const SimParameters& parameters)
{
	// TODO: implement other external forces

	// Gravity
	if (parameters.gravityEnabled) {
		node.force.y() -= parameters.gravityG * node.mass;
	}
}

void Solver::computeGridCollision(Node& node, const System& system, const SimParameters& parameters)
{
	// TODO: implement additional boundary conditions
	// TODO: consider more sophisticated collision response (ex. coefficients of restitution, friction)

	// enforce boundary conditions

	// non-elastic boundaries
	if (node.pos.x() < system.boundaryStart		// left wall
		|| node.pos.x() > system.boundaryEnd	// right wall
		|| node.pos.y() > system.boundaryEnd	// top wall (ceiling)
		|| node.pos.z() < system.boundaryStart  // back wall
		|| node.pos.z() > system.boundaryEnd) { // front wall
		node.vel.setZero();
	}

	// elastic boundary
	if (node.pos.y() < system.boundaryStart) { // bottom wall (floor)
		node.vel.y() = std::max(0.0, node.vel.y());
	}
}

void Solver::computeParticleCollision(Eigen::Vector3d& pos, Eigen::Vector3d& vel, const System& system, const SimParameters& parameters)
{
	// TODO: implement additional boundary conditions
	// TODO: consider more sophisticated collision response (ex. coefficients of restitution, friction)

	// enforce boundary conditions

	// non-elastic boundaries
	if (pos.x() < system.boundaryStart) // left wall
	{
		pos(0) = system.boundaryStart;
		vel(0) = 0;
	}

	if (pos.x() > system.boundaryEnd) // right wall
	{
		pos(0) = system.boundaryEnd;
		vel(0) = 0;
	}

	if (pos.z() < system.boundaryStart) // back wall
	{
		pos(2) = system.boundaryStart;
		vel(2) = 0;
	}

	if (pos.z() > system.boundaryEnd) // front wall
	{
		pos(2) = system.boundaryEnd;
		vel(2) = 0;
	}

	if (pos.y() > system.boundaryEnd) // top wall (ceiling)
	{
		pos(1) = system.boundaryEnd;
		vel(1) = 0;
	}

	if (pos.y() < system.boundaryStart) // bottom wall (floor)
	{
		pos(1) = system.boundaryStart;
		vel(1) = std::max(0.0, vel(1));
	}
}

void Solver::clock(unsigned int& current, unsigned int& total, std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds>& start)
{
	auto end	  = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	start		  = end;

	current = duration.count();
	total += current;
}

void Solver::computeTotalEnergy(Stats& stats, const System& system)
{
	stats.totalKineticEnergy   = 0;
	stats.totalPotentialEnergy = 0;

	// kinetic energy
	for (int pi = 0; pi < system.partCount; pi++) {
		const auto& mass = system.partMass[pi];
		const auto& vel  = system.partVel[pi];

		stats.totalKineticEnergy += 0.5 * mass * vel.squaredNorm();
	}

	// potential energy
	for (int pi = 0; pi < system.partCount; pi++) {
		const auto& model = system.partModel[pi];
		const auto& vol0  = system.partVol0[pi];
		const auto& F_E   = system.partF_E[pi];
		const auto& R_E   = system.partR_E[pi];
		const auto& F_P   = system.partF_P[pi];
		const auto& J_P   = system.partJ_P[pi];

		stats.totalPotentialEnergy += model->computePotentialEnergy(F_E, R_E, F_P, vol0);
	}
}

void Solver::additionalStats(Stats& stats, const System& system)
{
}
