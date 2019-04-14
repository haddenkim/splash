#include "ompSolver.h"
#include <omp.h>

using namespace Eigen;

void OmpSolver::advance(System& system, const SimParameters parameters, Stats& stats)
{
    // TODO: find out if this is a no-op when number has not changed, otherwise, need to build some control flow
    // set thread count
	omp_set_num_threads(parameters.numThreads);

	auto start = std::chrono::high_resolution_clock::now();

	resetGrid(system);
	clock(stats.timeReset, start);

	transferP2G(system, parameters);
	clock(stats.timeP2G, start);

	computeGrid(system, parameters);
	clock(stats.timeGrid, start);

	transferG2P(system, parameters);
	clock(stats.timeG2P, start);

	computeParticle(system, parameters);
	clock(stats.timePart, start);

	// log stats
	stats.simTime += parameters.timestep;
	stats.stepCount++;
}

void OmpSolver::resetGrid(System& system)
{
}

void OmpSolver::transferP2G(System& system, const SimParameters& parameters)
{
}

void OmpSolver::computeGrid(System& system, const SimParameters& parameters)
{
}

void OmpSolver::transferG2P(System& system, const SimParameters& parameters)
{
}

void OmpSolver::computeParticle(System& system, const SimParameters& parameters)
{
}