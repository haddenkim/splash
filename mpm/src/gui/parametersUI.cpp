#include "parametersUI.h"
#include <imgui/imgui.h>

ParametersUI::ParametersUI(SimParameters& simParameters)
	: simParameters_(simParameters)
{
}

void ParametersUI::draw()
{
	ImGui::Separator();
	ImGui::NewLine();
	ImGui::Combo("Solver method", &simParameters_.selectedSolver, simParameters_.solverNames, simParameters_.numSolvers);
	ImGui::InputFloat("Timestep", &simParameters_.timestep, 0.f, 0.f, "%.6f");
	ImGui::SliderInt("Num Steps", &simParameters_.numSteps, 1, 2000);

	ImGui::NewLine();
	ImGui::Checkbox("Gravity?", &simParameters_.gravityEnabled);
	ImGui::InputFloat("GravityG", &simParameters_.gravityG);

	ImGui::NewLine();
	ImGui::Text("Requires Reset");
	ImGui::SliderInt("Particle per Object", &simParameters_.particlesPerObject, 1, 100000);
	// ImGui::DragInt("Particle per Object", &simParameters_.particlesPerObject, 1.0F, 1, 10000);
	ImGui::SliderInt("Grid Size", &simParameters_.gridSize, 10, 64);

	ImGui::NewLine();
	ImGui::Text("Specific Solver Settings");

	ImGui::Text("Implicit");
	ImGui::SliderInt("Max Iters", &simParameters_.solveMaxIters, 1, 50);
	ImGui::SliderFloat("Tolerance", &simParameters_.solveTolerance, 0.f, 1.f, "%.4f");

	ImGui::Text("OpenMP");
	ImGui::SliderInt("Num Threads", &simParameters_.numThreads, 1, simParameters_.availThreads);
}