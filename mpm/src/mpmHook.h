#include "PhysicsHook.h"
#include "gui/ui.h"
#include "settings/renderSettings.h"
#include "settings/simParameters.h"
#include "settings/stats.h"
#include "state/particle.h"
#include "state/shape.h"
#include "state/system.h"
#include <Eigen/Core>
#include <initializer_list>

class Solver;

class MpmHook : public PhysicsHook {
public:
	MpmHook(std::initializer_list<Shape> initialShapes);

	void drawGUI() override;

	void initSimulation() override;

	void tick() override;

	bool simulateOneStep() override;

	void updateRenderGeometry() override;

	void renderRenderGeometry(igl::opengl::glfw::Viewer& viewer) override;

	void mouseClicked(double x, double y, int button) override;

	//
	void writePNG(igl::opengl::glfw::Viewer& viewer);
	Eigen::RowVector3d mapColor(double value, double base, double max);

	// libigl render data
	bool renderNeedsUpdate_;
	bool renderDataNeedsUpdates_;

	Eigen::MatrixXd particlePositions_;
	Eigen::MatrixXd particleColors_;
	Eigen::MatrixXd particleVelocities_;

	Eigen::MatrixXd gridPositions_;
	Eigen::MatrixXd gridVelocities_;
	Eigen::MatrixXd gridForces_;

	Eigen::MatrixXd gridBorders_;

	// simulation state
	std::vector<Shape> initialShapes_;
	System			   system_;

	// solvers
	std::vector<Solver*> solvers_;

	// settings + stats
	SimParameters  simParameters_;
	RenderSettings renderSettings_;
	Stats		   stats_;

	// ui
	UI ui_;
};
