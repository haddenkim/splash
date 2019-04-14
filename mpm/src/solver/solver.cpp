#include "solver.h"

using namespace Eigen;

void Solver::clock(int& dest, std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds>& start)
{
	auto end	  = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	start		  = end;

	dest = duration.count();
}

