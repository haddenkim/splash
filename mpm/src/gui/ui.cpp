#include "ui.h"
#include <imgui/imgui.h>

UI::UI(RenderSettings& renderSettings,
	   SimParameters&  simParameters,
	   Stats&		   stats,
	   System&		   system)
	: renderUI_(renderSettings)
	, parametersUI_(simParameters)
	, statsUI_(stats)
	, systemUI_(system)
{
	showParameters_ = true;
	showRender_		= false;
	showStats_		= false;
	showSystem_		= false;
}

void UI::draw()
{
	if (ImGui::CollapsingHeader("Additional UI", ImGuiTreeNodeFlags_DefaultOpen)) {
		ImGui::MenuItem("Sim Parameters", NULL, &showParameters_);
		ImGui::MenuItem("Renderer", NULL, &showRender_);
		ImGui::MenuItem("Stats", NULL, &showStats_);
		ImGui::MenuItem("System", NULL, &showSystem_);
	}

	if (showParameters_) {
		parametersUI_.draw();
	}
	if (showRender_) {
		renderUI_.draw();
	}
	if (showStats_) {
		statsUI_.draw();
	}
	if (showSystem_) {
		systemUI_.draw();
	}
}
