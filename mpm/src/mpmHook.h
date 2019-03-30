#include "PhysicsHook.h"

#include "gui/ui.h"
#include "settings/renderSettings.h"
#include "settings/simParameters.h"
#include "state/system.h"
#include "stats.h"

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
	Eigen::MatrixXd particlePositions_;
	Eigen::MatrixXd particleColors_;
	Eigen::MatrixXd particleVelocities_;

	Eigen::MatrixXd gridActivePositions_;
	Eigen::MatrixXd gridInactivePositions_;
	Eigen::MatrixXd gridActiveColors_;

	Eigen::MatrixXd meshV_;
	Eigen::MatrixXi meshF_;

	// settings
	RenderSettings renderSettings_;
	SimParameters  simParameters_;

	// simulation
	System  system_;
	Solver* solver_;

	// GUI
	UI ui_;

	// stats
	Stats stats_;
};