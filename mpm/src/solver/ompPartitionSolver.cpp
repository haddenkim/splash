#include "ompPartitionSolver.h"
#include <omp.h>

using namespace Eigen;

void OmpPartitionSolver::transferP2G(System& system, const SimParameters& parameters)
{
	// pre-compute particle values
	p2gPreComputeParts(system);

	// construct reduced list of active nodes
	p2gSetActiveNodes(system);

	// compute approx total particle-node interactions
	int x = 0;
#pragma omp parallel for reduction(+: x)
		for (int ni = 0; ni < activeNodes_.size(); ni++)
            x += activeNodes_[ni]->approxParts;

printf("approx total: %i\n", x);

	// transfer to node
	p2gComputeNodes(system);
}