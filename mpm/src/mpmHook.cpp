#include "mpmHook.h"
#include "solver/ompSolver.h"

// TODO clean up linking lodepng
#include "../../lib/lodepng/lodepng.h"

using namespace Eigen;

MpmHook::MpmHook(std::initializer_list<Shape> initialShapes)
	: PhysicsHook()
	, ui_(renderSettings_, simParameters_, stats_, system_)
{
	initialShapes_ = initialShapes;

	// available solvers
	// solvers_.emplace_back(new SerialImplicitCRSolver());
	solvers_.emplace_back(new OmpSolver());
	solvers_.emplace_back(new Solver());

	// set solver names for gui
	simParameters_.solverNames = new char*[solvers_.size()];
	simParameters_.numSolvers  = solvers_.size();
	for (int i = 0; i < solvers_.size(); i++) {

		std::string name = solvers_[i]->name();

		simParameters_.solverNames[i] = new char[name.size()];
		strncpy(simParameters_.solverNames[i], name.c_str(), name.size());
	}
}

void MpmHook::drawGUI()
{
	ui_.draw();
}

void MpmHook::initSimulation()
{
	renderDataNeedsUpdates_ = true;

	system_.reset(simParameters_.gridSize);
	stats_.reset();

	for(Solver* solver: solvers_)
	{
		solver->reset();
	}

	// rebuild system
	for (const Shape& shape : initialShapes_) {
		system_.addCube(simParameters_.particlesPerObject,
						shape.center,
						shape.velocity,
						shape.color);
	}
	system_.sortParticles();

	// bounds
	{
		double s = (double)system_.boundaryStart_ / system_.gridSize_; // start
		double e = (double)system_.boundaryEnd_ / system_.gridSize_;   // end

		// x = L(eft) or R(ight)
		// y = D(own) or U(p)
		// z = B(ack) or F(ront)
		RowVector3d LDB(s, s, s);
		RowVector3d RDB(e, s, s);
		RowVector3d LUB(s, e, s);
		RowVector3d RUB(e, e, s);

		RowVector3d LDF(s, s, e);
		RowVector3d RDF(e, s, e);
		RowVector3d LUF(s, e, e);
		RowVector3d RUF(e, e, e);

		gridBorders_.resize(12, 6); // 3 start , 3 end

		// x edges
		gridBorders_.row(0) << LDB, RDB;
		gridBorders_.row(1) << LUB, RUB;
		gridBorders_.row(2) << LDF, RDF;
		gridBorders_.row(3) << LUF, RUF;

		// y edges
		gridBorders_.row(4) << LDB, LUB;
		gridBorders_.row(5) << RDB, RUB;
		gridBorders_.row(6) << LDF, LUF;
		gridBorders_.row(7) << RDF, RUF;

		// z edges
		gridBorders_.row(8) << LDB, LDF;
		gridBorders_.row(9) << RDB, RDF;
		gridBorders_.row(10) << LUB, LUF;
		gridBorders_.row(11) << RUB, RUF;
	}

	// grid nodes
	{
		int nodeCount = system_.nodes_.size();
		gridPositions_.resize(nodeCount, 3);
		gridVelocities_.resize(nodeCount, 3);
		gridForces_.resize(nodeCount, 3);

		for (int ni = 0; ni < nodeCount; ni++) {
			const Node& node = system_.nodes_[ni];

			gridPositions_.block<1, 3>(ni, 0) = RowVector3d(node.x, node.y, node.z) / system_.gridSize_;
		}
	}
}

void MpmHook::tick()
{
}

bool MpmHook::simulateOneStep()
{
	solvers_[simParameters_.selectedSolver]->advance(system_, simParameters_, stats_);

	renderDataNeedsUpdates_ = true;

	if (simParameters_.numSteps == stats_.stepCount) {
		pause();
	}

	return false;
}

void MpmHook::updateRenderGeometry()
{
	if (stats_.stepCount % renderSettings_.drawInverval == 0 && renderDataNeedsUpdates_) {

		// particles
		{
			int psize = system_.particles_.size();
			particlePositions_.resize(psize, 3);
			particleColors_.resize(psize, 3);
			particleVelocities_.resize(psize, 3);

			for (int i = 0; i < psize; i++) {
				particlePositions_.block<1, 3>(i, 0) = system_.particles_[i].pos / system_.gridSize_;
				particleColors_.block<1, 3>(i, 0)	= system_.particles_[i].color;

				particleVelocities_.block<1, 3>(i, 0) = system_.particles_[i].vel;
			}

			// particlePositions_.resize(system_.partCount, 3);
			// particleColors_.resize(system_.partCount, 3);

			// particlePositions_ = system_.partPos;
			// particleColors_	= system_.partColor;
		}

		// grid
		{
			for (int ni = 0; ni < system_.nodes_.size(); ni++) {
				const Node& node = system_.nodes_[ni];

				gridVelocities_.block<1, 3>(ni, 0) = node.vel / system_.gridSize_;
				gridForces_.block<1, 3>(ni, 0)	 = node.force / system_.gridSize_;
			}

			// update flag
			renderDataNeedsUpdates_ = false;
			renderNeedsUpdate_		= true;
		}
	}
}

void MpmHook::renderRenderGeometry(igl::opengl::glfw::Viewer& viewer)
{
	viewer.data().point_size = renderSettings_.pointSize;
	viewer.data().line_width = renderSettings_.lineWidth;

	if (renderNeedsUpdate_ || renderSettings_.visibilityChanged) {

		viewer.data().clear();

		// particles
		if (renderSettings_.showParticles) {
			viewer.data().add_points(particlePositions_, particleColors_);
		}

		if (renderSettings_.showParticleVelocity) {
			RowVector3d red(1, 0, 0);
			viewer.data().add_edges(particlePositions_, particlePositions_ + (particleVelocities_ * renderSettings_.vectorScale), red);
		}

		if (renderSettings_.showBoundary) {
			// boundary
			RowVector3d red(1, 0, 0);
			MatrixXd	borderStart = gridBorders_.block<12, 3>(0, 0);
			MatrixXd	borderEnd   = gridBorders_.block<12, 3>(0, 3);

			viewer.data().add_edges(borderStart, borderEnd, red);
		}

		// grid
		if (renderSettings_.showGrid) {
			RowVector3d lightGrey(0.8, 0.8, 0.8);
			viewer.data().add_points(gridPositions_, lightGrey);
		}

		if (renderSettings_.showGridVelocity) {
			RowVector3d red(1, 0, 0);
			viewer.data().add_edges(gridPositions_, gridPositions_ + (gridVelocities_ * renderSettings_.vectorScale), red);
		}

		if (renderSettings_.showGridForce) {
			RowVector3d red(1, 0, 0);
			viewer.data().add_edges(gridPositions_, gridPositions_ + (gridForces_ * renderSettings_.vectorScale), red);
		}

		// write png only when sim changes, not if visibility settings change
		if (renderSettings_.writePNG && !isPaused() && renderNeedsUpdate_) {
			writePNG(viewer);
		}

		renderNeedsUpdate_				  = false;
		renderSettings_.visibilityChanged = false;
	}
}

void MpmHook::mouseClicked(double x, double y, int button)
{
}

void MpmHook::writePNG(igl::opengl::glfw::Viewer& viewer)
{
	static int pngCount = 0;
	int		   w		= 640;
	int		   h		= 400;

	// Allocate temporary buffers for image
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(w, h);
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(w, h);
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(w, h);
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(w, h);

	// Draw the scene in the buffers
	viewer.core.draw_buffer(viewer.data(), false, R, G, B, A);

	// padding filename
	std::string str;
	if (pngCount < 10)
		str = "00" + std::to_string(pngCount);
	else if (pngCount < 100)
		str = "0" + std::to_string(pngCount);
	else {
		str = std::to_string(pngCount);
	}

	std::string filename = "out/" + str + ".png";
	// igl::png::writePNG(R, G, B, A, filename); // broken at libigl

	// interleave buffer channels
	const int				   comp			   = 4;				  // 4 Channels Red, Green, Blue, Alpha
	const int				   stride_in_bytes = R.rows() * comp; // Length of one row in bytes
	std::vector<unsigned char> data(R.size() * comp, 0);		  // The image itself;

	for (unsigned i = 0; i < R.rows(); ++i) {
		for (unsigned j = 0; j < R.cols(); ++j) {
			data[(j * R.rows() * comp) + (i * comp) + 0] = R(i, R.cols() - 1 - j);
			data[(j * R.rows() * comp) + (i * comp) + 1] = G(i, R.cols() - 1 - j);
			data[(j * R.rows() * comp) + (i * comp) + 2] = B(i, R.cols() - 1 - j);
			data[(j * R.rows() * comp) + (i * comp) + 3] = A(i, R.cols() - 1 - j);
		}
	}

	// Encode the image
	unsigned error = lodepng::encode(filename, data, w, h);

	// TODO: handle errors
	printf("lodepng err: %ui\n", error);

	// for file name
	pngCount++;
}