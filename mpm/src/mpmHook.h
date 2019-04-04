#include "PhysicsHook.h"
#include "gui/ui.h"
#include "settings/renderSettings.h"
#include "settings/simParameters.h"
#include "settings/stats.h"
#include "state/particle.h"
#include "state/system.h"
#include <Eigen/Core>

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
	Eigen::MatrixXd particlePositions_;
	Eigen::MatrixXd particleColors_;

	Eigen::MatrixXd gridCorners_;
	Eigen::MatrixXd gridBorders_;

	// simulation state
	System system_;

	// settings + stats
	SimParameters  simParameters_;
	RenderSettings renderSettings_;
	Stats		   stats_;

	// ui
	UI ui_;
};
