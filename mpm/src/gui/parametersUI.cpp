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

	ImGui::Checkbox("Gravity?", &simParameters_.gravityEnabled);
	ImGui::InputFloat("GravityG", &simParameters_.gravityG);
}