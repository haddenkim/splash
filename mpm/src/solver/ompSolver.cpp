#include "ompSolver.h"
#include <omp.h>

using namespace Eigen;

void OmpSolver::advance(System& system, const SimParameters parameters, Stats& stats)
{
#pragma omp parallel
	{
		printf("Hello ");
		printf(" World \n");
	}
}