#include "Benchmark.h"

#include "solver/solverOmpGather.h"
#include "solver/solverOmpScatter.h"

#include "state/system.h"
#include "state/systemSetup.h"

#include <chrono>
#include <future>
#include <unistd.h>

Benchmark::Benchmark(SolverType type, SystemStart start, int argc, char* argv[])
	: start_(start)
{
	parseArgs(argc, argv);

	// set num threads
	omp_set_num_threads(simParameters_.numThreads);

#pragma omp parallel
	{
#pragma omp single
		printf("Number Threads: %i\n", omp_get_num_threads());
	}

	printf("-----Building System-----\n");
	// initialize system
	system_ = new System();

	// set system's initial state
	SystemSetup::initSystem(*system_, start_, simParameters_.particlesPerObject);

	// initialize solver
	switch (type) {
	case SolverType::SOLVER_OMP_GATHER:
		solver_ = new SolverOmpGather();
		break;

	case SolverType::SOLVER_OMP_SCATTER:
		solver_ = new SolverOmpScatter();
		break;

	default:
		assert(!"invalid type");
		break;
	}

	printf("Solver Type:     %s\n", solver_->name().c_str());
	printf("Total Particles: %i\n", system_->partCount);
	printf("Grid Dimensions: %i , %i , %i\n", WORLD_NUM_NODES_X, WORLD_NUM_NODES_Y, WORLD_NUM_NODES_Z);
}

void Benchmark::parseArgs(int argc, char* argv[])
{
	int  option;

	while ((option = getopt(argc, argv, "t:p:")) != -1)
		switch (option) {
		case 't':
			simParameters_.numThreads = atoi(optarg);
			break;

		case 'p':
			simParameters_.particlesPerObject = atoi(optarg);
			break;

		default:
			printf("Usage: [-t <n>] [-p <n>]\n");
			printf("t - number of threads (default = number of available processors)\n");
			printf("p - number of particles per object (default = 10 000)\n");

			exit(EXIT_FAILURE);
		}

}

void Benchmark::run(int numSteps)
{
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

	delete (system_);
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
	printf("Total Particles: %i\n", system_->partCount);
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