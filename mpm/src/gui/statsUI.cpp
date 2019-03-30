#include "statsUI.h"

#include <imgui/imgui.h>

using namespace ImGui;

StatsUI::StatsUI(const Stats& stats)
	: stats_(stats)
{
}

void StatsUI::draw()
{
	// Define next window position + size
	SetNextWindowPos(ImVec2(180, 10), ImGuiSetCond_FirstUseEver);
	SetNextWindowSize(ImVec2(200, 160), ImGuiSetCond_FirstUseEver);
	Begin("Stats", nullptr, ImGuiWindowFlags_NoSavedSettings);

	Text("Sim Time: %0.3f", stats_.simTime);

	End();
}