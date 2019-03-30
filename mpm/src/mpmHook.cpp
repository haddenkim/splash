#include "mpmHook.h"

#include "solver/serialSolver.h"
#include <imgui/imgui.h>
#include <state/node.h>
#include <state/particle.h>

using namespace Eigen;

MpmHook::MpmHook()
	: PhysicsHook()
	, system_(Vector3d(0.5, 0.5, 0.5), Vector3d(4, 4, 4))
	, ui_(renderSettings_, simParameters_, system_, stats_)
{
	solver_ = new SerialSolver();
}

void MpmHook::drawGUI()
{
	ImGui::Separator();

	// draw sub UIs
	ui_.draw();
}

void MpmHook::initSimulation()
{
	// clear system particles
	system_.clearParticles();


	double partMass = 1.0;
	double partVol  = 1.0;

	system_.addParticle(partMass, partVol, 1.2, 1.2, 1.2, 0.1, 0, 0);
	// system_.addParticle(partMass, partVol, 0.00, 0.75, 0, 0.1, 0, 0);
	// system_.addParticle(partMass, partVol, 0.25, 1.25, 0, 0.1, 0, 0);
	// system_.addParticle(partMass, partVol, 2.75, 2.75, 0, 0.1, 0, 0);

	// reset stats
	stats_.simTime = 0.f;
}

void MpmHook::tick()
{
}

bool MpmHook::simulateOneStep()
{

	solver_->simulateOneTick(system_, simParameters_);

	// update stats
	stats_.simTime += simParameters_.timestep;

	return false;
}

void MpmHook::updateRenderGeometry()
{
	// particles
	int psize = system_.particles_.size();
	particlePositions_.resize(psize, 3);
	for (int i = 0; i < psize; i++) {
		particlePositions_.block<1, 3>(i, 0) = system_.particles_[i].position;
	}

	particleColors_.resize(psize, 3);
	particleColors_.setConstant(1.0);

	// grid
	int gsize = system_.nodes_.size();
	gridPositions_.resize(gsize, 3);
	for (int i = 0; i < gsize; i++) {
		gridPositions_.block<1, 3>(i, 0) = system_.nodes_[i].position;
	}

	gridColors_.resize(gsize, 3);
	gridColors_.setConstant(0.0);

	// floor
	meshV_.resize(4, 3);
	meshV_ << 0, 0, 0,
		10, 0, 0,
		0, 0, 10,
		10, 0, 10;

	meshF_.resize(2, 3);
	meshF_ << 0, 3, 1,
		0, 2, 3;
}

void MpmHook::renderRenderGeometry(igl::opengl::glfw::Viewer& viewer)
{

	viewer.data().clear();

	// add particles
	if (renderSettings_.m_showParticles) {
		viewer.data().add_points(particlePositions_, particleColors_);
	}

	if (renderSettings_.m_showGrid) {
		viewer.data().point_size = 10.f;
		viewer.data().add_points(gridPositions_, gridColors_);
	}

	// floor
	if (renderSettings_.m_showFloor) {
		viewer.data().set_mesh(meshV_, meshF_);
	}
}

void MpmHook::mouseClicked(double x, double y, int button)
{
}
