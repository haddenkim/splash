#pragma once

#include <Eigen/Dense>

class System;

struct Shape {
	Shape(double cx, double cy, double cz, double vx, double vy, double vz, double r, double g, double b);

	Shape(Eigen::Vector3d center, Eigen::Vector3d velocity, Eigen::Vector3d color);

	Eigen::Vector3d center;
	Eigen::Vector3d velocity;
	Eigen::Vector3d color;

	void addTo(System& system, int partCount) const;
};
