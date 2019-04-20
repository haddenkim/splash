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

	if (stats_.stepCount != 0) {
		ImGui::NewLine();
		Columns(4);

		Text("Execution time (ms)\n");
		NextColumn();
		Text("Last Step\n");
		NextColumn();
		Text("Average\n");
		NextColumn();
		Text("Total\n");
		NextColumn();

		Text("Reset Grid");
		NextColumn();
		Text("%10i . %03i", stats_.timeReset / 1000, stats_.timeReset % 1000);
		NextColumn();
		int avgTimeReset = stats_.totTimeReset / stats_.stepCount;
		Text("%10i . %03i", avgTimeReset / 1000, avgTimeReset % 1000);
		NextColumn();
		Text("%10i . %03i", stats_.totTimeReset / 1000, stats_.totTimeReset % 1000);
		NextColumn();

		Text("P2G");
		NextColumn();
		Text("%10i . %03i", stats_.timeP2G / 1000, stats_.timeP2G % 1000);
		NextColumn();
		int avgTimeP2G = stats_.totTimeP2G / stats_.stepCount;
		Text("%10i . %03i", avgTimeP2G / 1000, avgTimeP2G % 1000);
		NextColumn();
		Text("%10i . %03i", stats_.totTimeP2G / 1000, stats_.totTimeP2G % 1000);
		NextColumn();

		Text("Grid Dynamics");
		NextColumn();
		Text("%10i . %03i", stats_.timeGrid / 1000, stats_.timeGrid % 1000);
		NextColumn();
		int avgTimeGrid = stats_.totTimeGrid / stats_.stepCount;
		Text("%10i . %03i", avgTimeGrid / 1000, avgTimeGrid % 1000);
		NextColumn();
		Text("%10i . %03i", stats_.totTimeGrid / 1000, stats_.totTimeGrid % 1000);
		NextColumn();

		Text("P2G");
		NextColumn();
		Text("%10i . %03i", stats_.timeG2P / 1000, stats_.timeG2P % 1000);
		NextColumn();
		int avgTimeG2P = stats_.totTimeG2P / stats_.stepCount;
		Text("%10i . %03i", avgTimeG2P / 1000, avgTimeG2P % 1000);
		NextColumn();
		Text("%10i . %03i", stats_.totTimeG2P / 1000, stats_.totTimeG2P % 1000);
		NextColumn();

		Text("Particle Update");
		NextColumn();
		Text("%10i . %03i", stats_.timePart / 1000, stats_.timePart % 1000);
		NextColumn();
		int avgTimePart = stats_.totTimePart / stats_.stepCount;
		Text("%10i . %03i", avgTimePart / 1000, avgTimePart % 1000);
		NextColumn();
		Text("%10i . %03i", stats_.totTimePart / 1000, stats_.totTimePart % 1000);
		NextColumn();

		NextColumn();
		NextColumn();
		NextColumn();
		NextColumn();

		Text("Total");
		NextColumn();
		int totalLastTime = stats_.timeReset + stats_.timeP2G + stats_.timeGrid + stats_.timeG2P + stats_.timePart;
		Text("%10i . %03i", totalLastTime / 1000, totalLastTime % 1000);
		NextColumn();
		int totalTime = stats_.totTimeReset + stats_.totTimeP2G + stats_.totTimeGrid + stats_.totTimeG2P + stats_.totTimePart;
		int avgTime   = totalTime / stats_.stepCount;
		Text("%10i . %03i", avgTime / 1000, avgTime % 1000);
		NextColumn();
		Text("%10i . %03i", totalTime / 1000, totalTime % 1000);
		NextColumn();

		Columns(1);
	}

	End();
}