#include "systemUI.h"
#include "state/node.h"
#include "state/particle.h"
#include "state/particleNodeLink.h"

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

	// Table headers
	Text("index");
	NextColumn();
	Text("position");
	NextColumn();
	Text("velocity");
	NextColumn();

	for (int i = 0; i < system_.particles_.size(); i++) {
		const Particle& particle = system_.particles_[i];

		Text("%i", i);
		NextColumn();
		Text("%0.2f %0.2f %0.2f", particle.position.x(), particle.position.y(), particle.position.z());
		NextColumn();
		Text("%0.2f %0.2f %0.2f", particle.velocity.x(), particle.velocity.y(), particle.velocity.z());
		NextColumn();
	}

	ImGui::Columns(1);
}

void SystemUI::displayNodeData()
{
	NewLine();
	Separator();
	Text("active nodes: %i", system_.activeNodes_);

	double			totMass = 0;
	Eigen::Vector3d totMomentum;
	totMomentum.setZero();

	ImGui::Columns(5);

	// Table headers
	Text("index");
	NextColumn();
	Text("position");
	NextColumn();
	Text("mass");
	NextColumn();
	Text("velocity");
	NextColumn();
	Text("force");
	NextColumn();

	for (int i = 0; i < system_.nodes_.size(); i++) {
		const Node& node = system_.nodes_[i];

		if (!node.active) {
			continue;
		}

		Text("%i", node.gridIndex);
		NextColumn();
		Text("%0.2f %0.2f %0.2f", node.position.x(), node.position.y(), node.position.z());
		NextColumn();
		Text("%0.3f", node.mass);
		NextColumn();
		Text("%0.2f %0.2f %0.2f", node.velocity.x(), node.velocity.y(), node.velocity.z());
		NextColumn();
		Text("%0.2f %0.2f %0.2f", node.force.x(), node.force.y(), node.force.z());
		NextColumn();

		totMass += node.mass;
		totMomentum += node.mass * node.velocity;
	}

	ImGui::Columns(1);

	NewLine();
	Text("total mass: %0.3f", totMass);
	Text("total momentum: %0.2f %0.2f %0.2f", totMomentum.x(), totMomentum.y(), totMomentum.z());
}

void SystemUI::displayLinkData()
{
}