#include "shape.h"
#include "settings/constants.h"
#include "state/particle.h"

using namespace Eigen;

Shape::Shape(ModelType type, double cx, double cy, double cz, double vx, double vy, double vz, double r, double g, double b)
	: type(type)
	, center(cx, cy, cz)
	, velocity(vx, vy, vz)
	, color(r, g, b)
{
	// scale to world dimension
	center.x() = cx * WORLD_NUM_NODES_X;
	center.y() = cy * WORLD_NUM_NODES_Y;
	center.z() = cz * WORLD_NUM_NODES_Z;

	velocity.x() = vx * WORLD_NUM_NODES_X;
	velocity.y() = vy * WORLD_NUM_NODES_Y;
	velocity.z() = vz * WORLD_NUM_NODES_Z;
};

// TODO: Allow for alternative shapes
// For now, shape is a cube
Eigen::Vector3d Shape::getRandomParticlePos()
{
	double diameter = (double)WORLD_NUM_NODES_X / 10;

	// random position from [-1,1]
	Vector3d position = Vector3d::Random().array();

	// scale and translate
	return position * diameter + center;
}
