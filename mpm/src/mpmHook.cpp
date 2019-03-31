#include "mpmHook.h"

#include "solver/serialSolver.h"
#include <imgui/imgui.h>
#include <state/node.h>
#include <state/particle.h>

using namespace Eigen;

MpmHook::MpmHook()
	: PhysicsHook()
	, system_(Vector3d(0.5, 0.5, 0.5), // cell size
			  Vector3d(10, 10, 10))	// world size
	, solver_(new SerialSolver)
	, ui_(solver_, renderSettings_, simParameters_, system_, stats_)
{
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

	// system_.addParticle(partMass, partVol, 1.2, 8.2, 1.2, 1, 0, 0);
	// system_.addParticle(partMass, partVol, 1.0, 8.4, 1.2, 1, 0, 0);

	// cube
	for (double z = 4; z <= 6; z += 0.2) {
		for (double y = 7; y <= 9; y += 0.2) {
			for (double x = 4; x < 6; x+= 0.2) {
				system_.addParticle(partMass, partVol, x, y, z, 0, 0, 0);
			}
		}
	}

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
	{
		int psize = system_.particles_.size();
		particlePositions_.resize(psize, 3);
		particleVelocities_.resize(psize, 3);

		for (int i = 0; i < psize; i++) {
			particlePositions_.block<1, 3>(i, 0)  = system_.particles_[i].position;
			particleVelocities_.block<1, 3>(i, 0) = system_.particles_[i].velocity;
		}

		particleColors_.resize(psize, 3);
		particleColors_.setConstant(1.0);
	}

	// grid
	{
		int gsize = system_.nodes_.size();
		gridActivePositions_.resize(system_.activeNodes_, 3);
		gridVelocities_.resize(system_.activeNodes_, 3);
		gridForces_.resize(system_.activeNodes_, 3);

		gridInactivePositions_.resize(gsize - system_.activeNodes_, 3);

		int activeNI   = 0;
		int inactiveNI = 0;

		for (const Node& node : system_.nodes_) {
			if (node.active) {
				gridActivePositions_.block<1, 3>(activeNI, 0) = node.position;
				gridVelocities_.block<1, 3>(activeNI, 0)	  = node.velocity;
				gridForces_.block<1, 3>(activeNI, 0)		  = node.force;
				activeNI++;
			} else {
				gridInactivePositions_.block<1, 3>(inactiveNI, 0) = node.position;
				inactiveNI++;
			}
		}

		gridActiveColors_.resize(system_.activeNodes_, 3);
		gridActiveColors_.setConstant(0.0);
	}

	// floor
	{
		meshV_.resize(4, 3);
		meshV_ << 0, 0, 0,
			10, 0, 0,
			0, 0, 10,
			10, 0, 10;

		meshF_.resize(2, 3);
		meshF_ << 0, 3, 1,
			0, 2, 3;
	}
}

void MpmHook::renderRenderGeometry(igl::opengl::glfw::Viewer& viewer)
{

	viewer.data().clear();

	viewer.data().point_size = renderSettings_.pointSize;
	viewer.data().line_width = renderSettings_.lineWidth;

	// particles
	if (renderSettings_.showParticles) {
		viewer.data().add_points(particlePositions_, particleColors_);
	}
	if (renderSettings_.showParticleVelocity) {
		const RowVector3d black(0, 0, 0);
		viewer.data().add_edges(particlePositions_, particlePositions_ + particleVelocities_, black);
	}

	// grid
	if (renderSettings_.showGrid) {
		const RowVector3d grey(0.5, 0.5, 0.5);
		viewer.data().add_points(gridInactivePositions_, grey);
	}
	if (renderSettings_.showActiveGrid || renderSettings_.showGrid) {
		viewer.data().add_points(gridActivePositions_, gridActiveColors_);
	}
	if (renderSettings_.showGridVelocity) {
		const RowVector3d black(0, 0, 0);
		viewer.data().add_edges(gridActivePositions_, gridActivePositions_ + gridVelocities_, black);
	}
	if (renderSettings_.showGridForce) {
		const RowVector3d red(1, 0, 0);
		viewer.data().add_edges(gridActivePositions_, gridActivePositions_ + gridForces_, red);
	}

	// floor
	if (renderSettings_.showFloor) {
		viewer.data().set_mesh(meshV_, meshF_);
	}
}

void MpmHook::mouseClicked(double x, double y, int button)
{
}
