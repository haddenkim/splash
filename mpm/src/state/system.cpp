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

void System::addCube(Vector3d center, Vector3d velocity, RowVector3d color)
{
	int	particleCount = 10000;
	double scale		 = 0.1;

	for (int i = 0; i < particleCount; i++) {

		Vector3d position = Vector3d::Random(3) * scale + center; // scale and translate

		particles_.emplace_back(Particle(position, velocity, color));
	}
}
