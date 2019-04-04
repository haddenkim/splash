#include "mpmHook.h"
#include "solver/serialSolver.h"

using namespace Eigen;

MpmHook::MpmHook()
	: PhysicsHook()
{
	// bounds
	{ // x = L(eft) or R(ight)
		// y = D(own) or U(p)
		// z = B(ack) or F(ront)
		RowVector3d LDB(0, 0, 0);
		RowVector3d RDB(1, 0, 0);
		RowVector3d LUB(0, 1, 0);
		RowVector3d RUB(1, 1, 0);

		RowVector3d LDF(0, 0, 1);
		RowVector3d RDF(1, 0, 1);
		RowVector3d LUF(0, 1, 1);
		RowVector3d RUF(1, 1, 1);

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
}

void MpmHook::drawGUI()
{
}

void MpmHook::initSimulation()
{
	step = 0;
	system_.clear();

	// falling blocks
	// {
	// 	system_.addCube(Vector3d(0.5, 0.4, 0.5),
	// 					Vector3d(0, 0, 0),
	// 					RowVector3d(1, 0, 1));

	// 	system_.addCube(Vector3d(0.4, 0.6, 0.5),
	// 					Vector3d(0, 0, 0),
	// 					RowVector3d(1, 1, 0));

	// 	system_.addCube(Vector3d(0.5, 0.8, 0.6),
	// 					  Vector3d(0, 0, 0),
	// 					  RowVector3d(0, 1, 0));
	// }

	// colliding blocks
	{
		system_.addCube(Vector3d(0.15, 0.8, 0.15),
						Vector3d(10, 0, 10),
						RowVector3d(0, 1, 1));

		system_.addCube(Vector3d(0.80, 0.7, 0.80),
						Vector3d(-10, 0, -10),
						RowVector3d(1, 1, 0));
	}

	// // block to wall
	// {
	// 	system_.addCube(Vector3d(0.5, 0.7, 0.5),
	// 					Vector3d(10, 0, 0),
	// 					RowVector3d(1, 1, 0));
	// }
}

void MpmHook::tick()
{
}

bool MpmHook::simulateOneStep()
{
	step++;

	SerialSolver::advance(system_, simParameters_);

	return false;
}

void MpmHook::updateRenderGeometry()
{
	// if (step % int(frame_dt / dt) == 0) {

	// particles
	{
		int psize = system_.particles_.size();
		particlePositions_.resize(psize, 3);
		particleColors_.resize(psize, 3);

		for (int i = 0; i < psize; i++) {
			particlePositions_.block<1, 3>(i, 0) = system_.particles_[i].pos;
			particleColors_.block<1, 3>(i, 0)	= system_.particles_[i].color;
		}

		// particlePositions_.resize(system_.partCount, 3);
		// particleColors_.resize(system_.partCount, 3);

		// particlePositions_ = system_.partPos;
		// particleColors_	= system_.partColor;
	}
	// }
}

void MpmHook::renderRenderGeometry(igl::opengl::glfw::Viewer& viewer)
{
	// if (step % int(frame_dt / dt) == 0) {

	viewer.data().clear();

	viewer.data().point_size = 3.0;

	// particles
	viewer.data().add_points(particlePositions_, particleColors_);

	// boundary
	RowVector3d red(1, 0, 0);
	MatrixXd	borderStart = gridBorders_.block<12, 3>(0, 0);
	MatrixXd	borderEnd   = gridBorders_.block<12, 3>(0, 3);

	viewer.data().add_edges(borderStart, borderEnd, red);
	// }
}

void MpmHook::mouseClicked(double x, double y, int button)
{
}
