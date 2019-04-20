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

	ImGui::Checkbox("Write PNG", &renderSettings_.writePNG);
	ImGui::SliderInt("Draw Interval (sim steps)", &renderSettings_.drawInverval, 1, 100);

	ImGui::SliderFloat("Point Size", &renderSettings_.pointSize, 0.1f, 20.0f);
	ImGui::SliderFloat("Line Width", &renderSettings_.lineWidth, 0.1f, 10.0f);
	ImGui::InputFloat("Vector Scaling", &renderSettings_.vectorScale, 0.f, 0.f, 5);

	ImGui::Text("Show:");
	if (ImGui::Checkbox("Particles", &renderSettings_.showParticles)) {
		renderSettings_.visibilityChanged = true;
	}
	if (ImGui::Checkbox("Particle Velocity", &renderSettings_.showParticleVelocity)) {
		renderSettings_.visibilityChanged = true;
	}

	ImGui::NewLine();
	if (ImGui::Checkbox("Active Grid", &renderSettings_.showActiveGrid)) {
		renderSettings_.visibilityChanged = true;
	}
	if (ImGui::Checkbox("Full Grid", &renderSettings_.showGrid)) {
		renderSettings_.visibilityChanged = true;
	}
	if (ImGui::Checkbox("Grid Velocity", &renderSettings_.showGridVelocity)) {
		renderSettings_.visibilityChanged = true;
	}
	if (ImGui::Checkbox("Grid Force", &renderSettings_.showGridForce)) {
		renderSettings_.visibilityChanged = true;
	}

	ImGui::NewLine();
	if (ImGui::Checkbox("World Boundary", &renderSettings_.showBoundary)) {
		renderSettings_.visibilityChanged = true;
	}
}