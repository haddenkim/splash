#include "debugUI.h"
#include <imgui/imgui.h>

DebugUI::DebugUI(Solver* solver, System& system, const SimParameters& parameters)
	: solver_(solver)
	, system_(system)
	, parameters_(parameters)
{
}

void DebugUI::draw()
{
	ImGui::Separator();
	ImGui::NewLine();

	ImGui::Text("Current Step: %s", SolverStepName[solver_->currentStep]);

	for (int i = 0; i < 9; i++) {
		bool targetAllowed = solver_->currentStep < i || solver_->currentStep == SolverStep::SOL_COMPLETE;

		if (ImGui::MenuItem(SolverStepName[i], NULL, !targetAllowed, targetAllowed)) {

			// target = SolverStep(i);
			solver_->advanceToStep(SolverStep(i), system_, parameters_);
		}
	}
}