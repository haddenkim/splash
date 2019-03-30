#include "renderUI.h"
#include <imgui/imgui.h>

RenderUI::RenderUI(RenderSettings& renderSettings)
	: renderSettings_(renderSettings)
{
}

void RenderUI::draw()
{
	ImGui::SliderFloat("Point Size", &renderSettings_.pointSize, 0.1f, 20.0f);
	ImGui::SliderFloat("Line Width", &renderSettings_.lineWidth, 0.1f, 10.0f);

	ImGui::Text("Show:");
	ImGui::Checkbox("Particles", &renderSettings_.showParticles);
	ImGui::Checkbox("Particle Velocity", &renderSettings_.showParticleVelocity);

	ImGui::NewLine();
	ImGui::Checkbox("Active Grid", &renderSettings_.showActiveGrid);
	ImGui::Checkbox("Full Grid", &renderSettings_.showGrid);

	ImGui::NewLine();
	ImGui::Checkbox("Floor", &renderSettings_.showFloor);
}