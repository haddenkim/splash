#include "shape.h"
#include "state/system.h"

using namespace Eigen;

Shape::Shape(double cx, double cy, double cz, double vx, double vy, double vz, double r, double g, double b)
	: center(cx, cy, cz)
	, velocity(vx, vy, vz)
	, color(r, g, b){

	};

Shape::Shape(Eigen::Vector3d center,
			 Eigen::Vector3d velocity,
			 Eigen::Vector3d color)
	: center(center)
	, velocity(velocity)
	, color(color){

	};

void Shape::addTo(System& system, int partCount) const
{

	double diameter = (double)system.gridSize_ / 10;

	for (int i = 0; i < partCount; i++) {
		// random position from [-1,1]
		Vector3d position = Vector3d::Random().array();

		position = position * diameter + center * system.gridSize_;

		system.addPart(position, velocity, color);
	}
}