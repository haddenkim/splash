#include "systemUI.h"
#include "state/node.h"
#include "state/particle.h"

#include <Eigen/Core>
#include <imgui/imgui.h>

using namespace ImGui;

SystemUI::SystemUI(const System& system)
	: system_(system)
{
}

void SystemUI::draw()
{
	// Define next window position + size
	SetNextWindowPos(ImVec2(180, 10), ImGuiSetCond_FirstUseEver);
	SetNextWindowSize(ImVec2(200, 160), ImGuiSetCond_FirstUseEver);
	Begin("System", nullptr, ImGuiWindowFlags_NoSavedSettings);

	Checkbox("Particles", &showParticles_);
	Checkbox("Nodes", &showNodes_);
	Checkbox("Links", &showLinks_);

	if (showParticles_) {
		displayParticleData();
	}
	if (showNodes_) {
		displayNodeData();
	}

	if (showLinks_) {
		displayLinkData();
	}

	End();
}

void SystemUI::displayParticleData()
{
	NewLine();
	Separator();
	Columns(3);

	// // Table headers
	// Text("index");
	// NextColumn();
	// Text("position");
	// NextColumn();
	// Text("velocity");
	// NextColumn();

	// for (int i = 0; i < system_.particles_.size(); i++) {
	// 	const Particle& particle = system_.particles_[i];

	// 	Text("%i", i);
	// 	NextColumn();
	// 	Text("%0.2f %0.2f %0.2f", particle.pos.x(), particle.pos.y(), particle.pos.z());
	// 	NextColumn();
	// 	Text("%0.2f %0.2f %0.2f", particle.vel.x(), particle.vel.y(), particle.vel.z());
	// 	NextColumn();
	// }

	ImGui::Columns(1);
}

void SystemUI::displayNodeData()
{
	NewLine();
	Separator();

}

void SystemUI::displayLinkData()
{
}