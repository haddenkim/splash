#include "ui.h"
#include <imgui/imgui.h>

UI::UI(RenderSettings& renderSettings, SimParameters& simParameters, const System& system)
	: parametersUI_(simParameters)
	, renderUI_(renderSettings)
	, statsUI_()
	, systemUI_(system)
{
	showParameters_ = false;
	showRender_		= false;
	showStats_		= false;
	showSystem_		= false;
}

void UI::draw()
{
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

	if (ImGui::BeginMainMenuBar()) {
		if (ImGui::BeginMenu("Windows")) {
			ImGui::MenuItem("Sim Parameters", NULL, &showParameters_);
			ImGui::MenuItem("Renderer", NULL, &showRender_);
			ImGui::MenuItem("Stats", NULL, &showStats_);
			ImGui::MenuItem("System", NULL, &showSystem_);

			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();
	}
}
