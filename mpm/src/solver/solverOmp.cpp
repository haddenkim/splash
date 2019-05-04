#include "solver/solverOmp.h"
#include "settings/simParameters.h"
#include <omp.h>

using namespace Eigen;

// void SolverOmp::advance(System& system, const SimParameters parameters, Stats& stats, int numSteps)
// {
// 	// TODO: find out if this is a no-op when number has not changed, otherwise, need to build some control flow
// 	// set thread count
// 	omp_set_num_threads(parameters.numThreads);

// 	Solver::advance(system, parameters, stats, numSteps);
// }