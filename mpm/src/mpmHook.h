#include "PhysicsHook.h"
#include "gui/ui.h"
#include "settings/renderSettings.h"
#include "settings/simParameters.h"
#include "settings/stats.h"
#include "solver/serialImplicitSolver.h"
#include "solver/serialSolver.h"
#include "state/particle.h"
#include "state/system.h"
#include <Eigen/Core>

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

	void writePNG(igl::opengl::glfw::Viewer& viewer);

	// libigl render data
	bool			renderNeedsUpdate_;
	Eigen::MatrixXd particlePositions_;
	Eigen::MatrixXd particleColors_;
	Eigen::MatrixXd particleVelocities_;

	Eigen::MatrixXd gridPositions_;
	Eigen::MatrixXd gridVelocities_;
	Eigen::MatrixXd gridForces_;

	Eigen::MatrixXd gridBorders_;

	// simulation state
	System system_;

	// solvers
	SerialSolver		 serialExplicitSolver_;
	SerialImplicitSolver serialImplicitSolver_;

	// settings + stats
	SimParameters  simParameters_;
	RenderSettings renderSettings_;
	Stats		   stats_;

	// ui
	UI ui_;
};
