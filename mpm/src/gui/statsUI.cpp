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

	Text("Step count: %i", stats_.stepCount);
	Text("Sim time: %0.4f", stats_.simTime);

	ImGui::NewLine();
	Text("Execution time (ms) of last step\n");
	Columns(2);

	Text("Reset Grid");
	NextColumn();
	Text("%10i . %03i", stats_.timeReset / 1000, stats_.timeReset % 1000);

	NextColumn();

	Text("P2G");
	NextColumn();
	Text("%10i . %03i", stats_.timeP2G / 1000, stats_.timeP2G % 1000);
	NextColumn();

	Text("Grid Dynamics");
	NextColumn();
	Text("%10i . %03i", stats_.timeGrid / 1000, stats_.timeGrid % 1000);
	NextColumn();

	Text("P2G");
	NextColumn();
	Text("%10i . %03i", stats_.timeG2P / 1000, stats_.timeG2P % 1000);
	NextColumn();

	Text("Particle Update");
	NextColumn();
	Text("%10i . %03i", stats_.timePart / 1000, stats_.timePart % 1000);
	NextColumn();

	Columns(1);

	End();
}