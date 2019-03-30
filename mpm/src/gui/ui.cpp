#include "ui.h"
#include <imgui/imgui.h>

UI::UI(RenderSettings& renderSettings, SimParameters& simParameters)
	: m_parametersUI(simParameters)
	, m_renderUI(renderSettings)
	, m_statsUI()
{
	m_showParameters = false;
	m_showRender	 = false;
	m_showStats		 = false;
}

void UI::draw()
{
	if (m_showParameters) {
		m_parametersUI.draw();
	}
	if (m_showRender) {
		m_renderUI.draw();
	}
	if (m_showStats) {
		m_statsUI.draw();
	}

	if (ImGui::BeginMainMenuBar()) {
		if (ImGui::BeginMenu("Windows")) {
			ImGui::MenuItem("Sim Parameters", NULL, &m_showParameters);
			ImGui::MenuItem("Renderer", NULL, &m_showRender);
			ImGui::MenuItem("Stats", NULL, &m_showStats);

			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();
	}
}
