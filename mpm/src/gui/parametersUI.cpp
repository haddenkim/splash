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

	ImGui::InputFloat("Timestep", &simParameters_.timestep, 0.f, 0.f, "%.6f");
	ImGui::SliderInt("Particle per Object", &simParameters_.particlesPerObject, 1, 100000);

	ImGui::Checkbox("Gravity?", &simParameters_.gravityEnabled);
	ImGui::InputFloat("GravityG", &simParameters_.gravityG);

	ImGui::Checkbox("Implicit Step", &simParameters_.solveImplicit);
}