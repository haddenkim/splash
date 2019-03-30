#include "PhysicsHook.h"

#include "settings/renderSettings.h"
#include "settings/simParameters.h"
#include "state/system.h"
#include "gui/ui.h"

class Solver;

class MpmHook : public PhysicsHook {
public:
	MpmHook();

	void drawGUI() override;

	void initSimulation() override;

	void tick() override;

	bool simulateOneStep() override;

	void updateRenderGeometry() override;

	void renderRenderGeometry(igl::opengl::glfw::Viewer& viewer) override;

	void mouseClicked(double x, double y, int button) override;

	// libigl render data
	Eigen::MatrixXd particlePositions;
	Eigen::MatrixXd particleColors;
	Eigen::MatrixXd gridPositions;
	Eigen::MatrixXd gridColors;
	Eigen::MatrixXd meshV;
	Eigen::MatrixXi meshF;

	// settings
	RenderSettings m_renderSettings;
	SimParameters m_simParameters;

	// simulation
	System m_system;
	Solver* m_solver;

	// GUI
	UI m_ui;
};