#pragma once

#include <Eigen/Dense>

struct Shape {
	Shape(double cx, double cy, double cz, double vx, double vy, double vz, double r, double g, double b)
		: center(cx, cy, cz)
		, velocity(vx, vy, vz)
		, color(r, g, b){

		};

	Shape(Eigen::Vector3d center,
		  Eigen::Vector3d velocity,
		  Eigen::Vector3d color)
		: center(center)
		, velocity(velocity)
		, color(color){

		};

	Eigen::Vector3d center;
	Eigen::Vector3d velocity;
	Eigen::Vector3d color;
};
