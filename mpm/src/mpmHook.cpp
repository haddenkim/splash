#include "mpmHook.h"

#include "solver/serialSolver.h"
#include <imgui/imgui.h>
#include <state/node.h>
#include <state/particle.h>

using namespace Eigen;

MpmHook::MpmHook()
	: PhysicsHook()
	, m_system(Vector3d(0.5, 0.5, 0.5), Vector3d(10, 10, 0))
	, m_ui(m_renderSettings, m_simParameters, m_system)
{
	m_solver = new SerialSolver();
}

void MpmHook::drawGUI()
{
	ImGui::Separator();

	// draw sub UIs
	m_ui.draw();
}

void MpmHook::initSimulation()
{
	double partMass = 1.0;
	double partVol  = 1.0;

	m_system.addParticle(partMass, partVol, 0, 0, 0, 1, 0, 0);
	m_system.addParticle(partMass, partVol, 0, 0.75, 0, 1, 0, 0);
	m_system.addParticle(partMass, partVol, 0.25, 1.25, 0, 1, 0, 0);
	m_system.addParticle(partMass, partVol, 2.75, 2.75, 0, 1, 0, 0);
}

void MpmHook::tick()
{
}

bool MpmHook::simulateOneStep()
{

	m_solver->simulateOneTick(m_system, m_simParameters);
}

void MpmHook::updateRenderGeometry()
{
	// particles
	int psize = m_system.particles_.size();
	particlePositions.resize(psize, 3);
	for (int i = 0; i < psize; i++) {
		particlePositions.block<1, 3>(i, 0) = m_system.particles_[i].position;
	}

	particleColors.resize(psize, 3);
	particleColors.setConstant(1.0);

	// grid
	int gsize = m_system.nodes_.size();
	gridPositions.resize(gsize, 3);
	for (int i = 0; i < gsize; i++) {
		gridPositions.block<1, 3>(i, 0) = m_system.nodes_[i].position;
	}

	gridColors.resize(gsize, 3);
	gridColors.setConstant(0.0);

	// floor
	meshV.resize(4, 3);
	meshV << 0, 0, 0,
		10, 0, 0,
		0, 0, 10,
		10, 0, 10;

	meshF.resize(2, 3);
	meshF << 0, 3, 1,
		0, 2, 3;
}

void MpmHook::renderRenderGeometry(igl::opengl::glfw::Viewer& viewer)
{

	viewer.data().clear();

	// add particles
	if (m_renderSettings.m_showParticles) {
		viewer.data().add_points(particlePositions, particleColors);
	}

	if (m_renderSettings.m_showGrid) {
		viewer.data().point_size = 10.f;
		viewer.data().add_points(gridPositions, gridColors);
	}

	// floor
	if (m_renderSettings.m_showFloor) {
		viewer.data().set_mesh(meshV, meshF);
	}
}

void MpmHook::mouseClicked(double x, double y, int button)
{
}
