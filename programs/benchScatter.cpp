#include "app/Benchmark.h"
#include "models/modelType.h"
#include "settings/systemStart.h"
#include "state/shape.h"

int main(int argc, char* argv[])
{
	SystemStart start;
	start.shapes.emplace_back(Shape(MODEL_SNOW, 0.2, 0.8, 0.2, 0.5, 0, 0.5, 0, 1, 1));
	start.shapes.emplace_back(Shape(MODEL_SNOW, 0.8, 0.7, 0.5, -0.5, 0, 0, 1, 0, 1));

	Benchmark benchmark(SolverType::SOLVER_OMP_SCATTER, start, argc, argv);

	benchmark.run(1000);
}
