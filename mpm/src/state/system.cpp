#include "system.h"
#include <Eigen/Dense>

using namespace Eigen;

System::System()
	: constitutiveModel_()
{
}

void System::clear()
{
	particles_.clear();
}

void System::addCube(int partCount, Vector3d center, Vector3d velocity, RowVector3d color)
{
	double scale		 = 0.1;

	for (int i = 0; i < partCount; i++) {

		Vector3d position = Vector3d::Random(3) * scale + center; // scale and translate

		particles_.emplace_back(Particle(position, velocity, color));
	}
}
