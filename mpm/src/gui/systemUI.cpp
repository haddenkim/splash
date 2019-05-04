#include "systemUI.h"
#include "state/node.h"
#include "state/particle.h"

#include <Eigen/Core>
#include <imgui/imgui.h>

using namespace ImGui;

SystemUI::SystemUI(const System& system)
	: system_(system)
{
	showParticles_ = true;
}

void SystemUI::draw()
{
	// Define next window position + size
	SetNextWindowPos(ImVec2(180, 10), ImGuiSetCond_FirstUseEver);
	SetNextWindowSize(ImVec2(200, 160), ImGuiSetCond_FirstUseEver);
	Begin("System", nullptr, ImGuiWindowFlags_NoSavedSettings);

	// Checkbox("Particles", &showParticles_);
	// Checkbox("Nodes", &showNodes_);
	// Checkbox("Links", &showLinks_);

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

	static int pi = 0;

	ImGui::InputInt("Particle", &pi);

	if (pi >= 0 && pi < system_.partCount) {
		Text("dataIndex\t%i", system_.particles[pi].dataIndex);
		Text("blockIndex\t%i", system_.particles[pi].blockIndex);
		NewLine();

		Text("position\t%0.2f\t %0.2f\t %0.2f", system_.partPos[pi].x(), system_.partPos[pi].y(), system_.partPos[pi].z());
		Text("velocity\t%0.2f\t %0.2f\t %0.2f", system_.partVel[pi].x(), system_.partVel[pi].y(), system_.partVel[pi].z());
		Text("B\t");
		Text("%0.2f\t %0.2f\t %0.2f\n", system_.partB[pi](0, 0), system_.partB[pi](0, 1), system_.partB[pi](0, 2));
		Text("%0.2f\t %0.2f\t %0.2f\n", system_.partB[pi](1, 0), system_.partB[pi](1, 1), system_.partB[pi](1, 2));
		Text("%0.2f\t %0.2f\t %0.2f\n", system_.partB[pi](2, 0), system_.partB[pi](2, 1), system_.partB[pi](2, 2));
		NewLine();

		Text("PE\t%i", system_.partModel[pi]->computePotentialEnergy());
		auto VPFT = system_.partModel[pi]->computeVolCauchyStress();
		Text("VolStress");
		Text("%0.2f\t %0.2f\t %0.2f\n", VPFT(0, 0), VPFT(0, 1), VPFT(0, 2));
		Text("%0.2f\t %0.2f\t %0.2f\n", VPFT(1, 0), VPFT(1, 1), VPFT(1, 2));
		Text("%0.2f\t %0.2f\t %0.2f\n", VPFT(2, 0), VPFT(2, 1), VPFT(2, 2));
		NewLine();

		Text("partVelGrad");
		Text("%0.2f\t %0.2f\t %0.2f\n", system_.partVelGrad[pi](0, 0), system_.partVelGrad[pi](0, 1), system_.partVelGrad[pi](0, 2));
		Text("%0.2f\t %0.2f\t %0.2f\n", system_.partVelGrad[pi](1, 0), system_.partVelGrad[pi](1, 1), system_.partVelGrad[pi](1, 2));
		Text("%0.2f\t %0.2f\t %0.2f\n", system_.partVelGrad[pi](2, 0), system_.partVelGrad[pi](2, 1), system_.partVelGrad[pi](2, 2));
		NewLine();

		Text("Model specific");
		Text(system_.partModel[pi]->getGui().c_str());

	} else {
		Text("invalid particle id");
	}

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