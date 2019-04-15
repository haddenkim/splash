#include "solver.h"

using namespace Eigen;

void Solver::clock(int& current, int& total, std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds>& start)
{
	auto end	  = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	start		  = end;

	current = duration.count();
	total += current;
}

