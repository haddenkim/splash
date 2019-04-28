#include "Benchmark.h"

#include "solver/ompAtomicSolver.h"
#include "solver/ompGatherSolver.h"

#include "state/system.h"
#include <chrono>
#include <future>

Benchmark::Benchmark(SolverType type, SystemStart start)
	: start_(start)
{
	printf("-----Building System-----\n");
	// initialize system
	system_ = new System();

	// set system's initial state
	system_->restart(start_);

	// initialize solver
	switch (type) {
	case SolverType::SOLVER_OMP_GATHER:
		solver_ = new OmpGatherSolver();
		break;

	case SolverType::SOLVER_OMP_ATOMIC:
		solver_ = new OmpAtomicSolver();
		break;

	default:
		solver_ = new Solver();
		break;
	}
}

void Benchmark::run(int numSteps)
{
	// debug
	numSteps = 10;

	printf("-----Running Simulation-----\n");

	stats_.reset();

	auto future = std::async(std::launch::async, [&] {
		solver_->advance(*system_, simParameters_, stats_, numSteps);

		return;
	});

	// print while running
	int					 elapsedTime = 0;
	std::chrono::seconds interval(1);

	while (future.wait_for(interval) == std::future_status::timeout) {
		elapsedTime++;

		printf("%i seconds | Step (of %i): %i\n", elapsedTime, numSteps, stats_.stepCount);
	}

	// cleanup
	future.get();

	printStats();
}

void Benchmark::printStats()
{
	unsigned int totalTime = stats_.totTimeReset
		+ stats_.totTimeP2G
		+ stats_.totTimeGrid
		+ stats_.totTimeG2P
		+ stats_.totTimePart;

	printf("\n");
	printf("-----Sim Complete-----\n");

	printf("Solver Type:     %s\n", solver_->name().c_str());
	printf("Total Particles: %i\n", system_->partCount());
	printf("Grid Dimensions: %i , %i , %i\n", WORLD_NUM_NODES_X, WORLD_NUM_NODES_Y, WORLD_NUM_NODES_Z);

	printf("\n");
	printf("Total Steps: %i\n", stats_.stepCount);
	printf("Total Sim Time: %f\n", stats_.simTime);

	printf("\n");
	printf("Total Execution Time(ms): %0.3f\n", (float)totalTime / 1000);
	printf("Avg   Execution Time(ms):  %0.3f\n", (float)totalTime / (float)stats_.stepCount / 1000);

	printf("\n");
	printf("Total Reset:\t %0.3f\n", (float)stats_.totTimeReset / 1000);
	printf("Total P2G:\t %0.3f\n", (float)stats_.totTimeP2G / 1000);
	printf("Total Grid:\t %0.3f\n", (float)stats_.totTimeGrid / 1000);
	printf("Total G2P:\t %0.3f\n", (float)stats_.totTimeG2P / 1000);
	printf("Total Part:\t %0.3f\n", (float)stats_.totTimePart / 1000);

	printf("\n");
	printf("Avg   Reset:\t %0.3f\n", (float)stats_.totTimeReset / (float)stats_.stepCount / 1000);
	printf("Avg   P2G:\t %0.3f\n", (float)stats_.totTimeP2G / (float)stats_.stepCount / 1000);
	printf("Avg   Grid:\t %0.3f\n", (float)stats_.totTimeGrid / (float)stats_.stepCount / 1000);
	printf("Avg   G2P:\t %0.3f\n", (float)stats_.totTimeG2P / (float)stats_.stepCount / 1000);
	printf("Avg   Part:\t %0.3f\n", (float)stats_.totTimePart / (float)stats_.stepCount / 1000);
}