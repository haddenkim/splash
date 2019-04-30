#pragma once
#include "models/modelType.h"
#include <Eigen/Dense>

// TODO: Allow for alternative shapes

struct Shape {
	Shape(ModelType type, double cx, double cy, double cz, double vx, double vy, double vz, double r, double g, double b);

	Eigen::Vector3d getRandomParticlePos() const;

	ModelType		type;
	Eigen::Vector3d center;
	Eigen::Vector3d velocity;
	Eigen::Vector3d color;
};
