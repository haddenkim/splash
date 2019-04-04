#include "renderUI.h"
#include <imgui/imgui.h>

RenderUI::RenderUI(RenderSettings& renderSettings)
	: renderSettings_(renderSettings)
{
}

void RenderUI::draw()
{
	ImGui::Separator();
	ImGui::NewLine();

	ImGui::SliderInt("Draw Interval (sim steps)", &renderSettings_.drawInverval, 1, 100);

	ImGui::SliderFloat("Point Size", &renderSettings_.pointSize, 0.1f, 20.0f);
	ImGui::SliderFloat("Line Width", &renderSettings_.lineWidth, 0.1f, 10.0f);

	ImGui::Text("Show:");
	ImGui::Checkbox("Particles", &renderSettings_.showParticles);
	ImGui::Checkbox("Particle Velocity", &renderSettings_.showParticleVelocity);

	ImGui::NewLine();
	ImGui::Checkbox("Active Grid", &renderSettings_.showActiveGrid);
	ImGui::Checkbox("Full Grid", &renderSettings_.showGrid);
	ImGui::Checkbox("Grid Velocity", &renderSettings_.showGridVelocity);
	ImGui::Checkbox("Grid Force", &renderSettings_.showGridForce);

	ImGui::NewLine();
	ImGui::Checkbox("World Boundary", &renderSettings_.showBoundary);
}