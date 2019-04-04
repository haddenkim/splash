#include "mpmHook.h"
#include "solver/serialImplicitSolver.h"
#include "solver/serialSolver.h"
#include "solver/solver.h"

// TODO clean up linking lodepng
#include "../../lib/lodepng/lodepng.h"

using namespace Eigen;

MpmHook::MpmHook()
	: PhysicsHook()
	, ui_(renderSettings_, simParameters_, stats_, system_)
{
	// bounds
	{
		double s = system_.boundary;	 // start
		double e = 1 - system_.boundary; // end

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
		int gsize = system_.gridSize * system_.gridSize * system_.gridSize;
		gridPositions_.resize(gsize, 3);
		gridVelocities_.resize(gsize, 3);
		gridForces_.resize(gsize, 3);

		int gi = 0;
		for (int i = 0; i < system_.gridSize; i++) {
			for (int j = 0; j < system_.gridSize; j++) {
				for (int k = 0; k < system_.gridSize; k++) {

					Vector3d nodePosition			  = Vector3d(i, j, k);
					gridPositions_.block<1, 3>(gi, 0) = nodePosition;

					gi++;
				}
			}
		}

		// scale
		gridPositions_ *= system_.dx;
	}
}

void MpmHook::drawGUI()
{
	ui_.draw();
}

void MpmHook::initSimulation()
{
	system_.clear();
	stats_.reset();

	// solver
	solver_ = new SerialSolver();
	// solver_ = new SerialImplicitSolver();

	// TODO Engineer process to modify at run time

	// // falling blocks
	// {
	// 	system_.addCube(Vector3d(0.5, 0.4, 0.5),
	// 					Vector3d(0, 0, 0),
	// 					RowVector3d(1, 0, 1));

	// 	system_.addCube(Vector3d(0.4, 0.6, 0.5),
	// 					Vector3d(0, 0, 0),
	// 					RowVector3d(0, 1, 1));

	// 	system_.addCube(Vector3d(0.6, 0.8, 0.5),
	// 					  Vector3d(0, 0, 0),
	// 					  RowVector3d(0, 1, 0));
	// }

	// colliding blocks
	// {
	// 	system_.addCube(Vector3d(0.2, 0.8, 0.2),
	// 					Vector3d(10, 0, 10),
	// 					RowVector3d(0, 1, 1));

	// 	system_.addCube(Vector3d(0.8, 0.7, 0.5),
	// 					Vector3d(-10, 0, 0),
	// 					RowVector3d(1, 0, 1));
	// }

	// block to wall
	{
		system_.addCube(Vector3d(0.5, 0.7, 0.5),
						Vector3d(30, 0, 0),
						RowVector3d(0, 1, 0));
	}

	renderNeedsUpdate_ = true;
}

void MpmHook::tick()
{
}

bool MpmHook::simulateOneStep()
{
	solver_->advance(system_, simParameters_, stats_);

	renderNeedsUpdate_ = true;

	return false;
}

void MpmHook::updateRenderGeometry()
{
	if (stats_.stepCount % renderSettings_.drawInverval == 0 && renderNeedsUpdate_) {

		// particles
		{
			int psize = system_.particles_.size();
			particlePositions_.resize(psize, 3);
			particleColors_.resize(psize, 3);
			particleVelocities_.resize(psize, 3);

			for (int i = 0; i < psize; i++) {
				particlePositions_.block<1, 3>(i, 0) = system_.particles_[i].pos;
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
			int gi = 0;
			for (int i = 0; i < system_.gridSize; i++) {
				for (int j = 0; j < system_.gridSize; j++) {
					for (int k = 0; k < system_.gridSize; k++) {
						const Node& node = system_.nodes_[i][j][k];

						gridVelocities_.block<1, 3>(gi, 0) = node.vel;
						gridForces_.block<1, 3>(gi, 0)	 = node.force;

						gi++;
					}
				}
			}
		}

		// update flag
		renderNeedsUpdate_ = false;
	}
}

void MpmHook::renderRenderGeometry(igl::opengl::glfw::Viewer& viewer)
{
	viewer.data().point_size = renderSettings_.pointSize;
	viewer.data().line_width = renderSettings_.lineWidth;

	if (stats_.stepCount % renderSettings_.drawInverval == 0) {

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
		if (renderSettings_.showGridVelocity) {
			RowVector3d red(1, 0, 0);
			viewer.data().add_edges(gridPositions_, gridPositions_ + (gridVelocities_ * renderSettings_.vectorScale), red);
		}

		if (renderSettings_.showGridForce) {
			RowVector3d red(1, 0, 0);
			viewer.data().add_edges(gridPositions_, gridPositions_ + (gridForces_ * renderSettings_.vectorScale), red);
		}

		// write png
		if (renderSettings_.writePNG && !isPaused()) {
			writePNG(viewer);
		}
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