#include "renderUI.h"
#include <imgui/imgui.h>

RenderUI::RenderUI(RenderSettings& renderSettings)
	: m_renderSettings(renderSettings)
{
}

void RenderUI::draw()
{
	ImGui::Checkbox("Show Particles", &m_renderSettings.m_showParticles);
	ImGui::Checkbox("Show Grid", &m_renderSettings.m_showGrid);
	ImGui::Checkbox("Show Floor", &m_renderSettings.m_showFloor);
}