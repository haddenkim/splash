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
	ImGui::Combo("Solver method", (int*)&simParameters_.solveMethod, "Explicit\0Implicit\0OpenMP\0\0");
	ImGui::InputFloat("Timestep", &simParameters_.timestep, 0.f, 0.f, "%.6f");

	ImGui::NewLine();
	ImGui::Checkbox("Gravity?", &simParameters_.gravityEnabled);
	ImGui::InputFloat("GravityG", &simParameters_.gravityG);


	ImGui::NewLine();
	ImGui::Text("Requires Reset");
	ImGui::SliderInt("Particle per Object", &simParameters_.particlesPerObject, 1, 100000);
}