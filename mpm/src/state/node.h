#pragma once
#include <Eigen/Core>
#include <vector>

struct Particle;

struct Node {
	// time integration bookkeeping
	double			mass; // mass
	Eigen::Vector3d vel;  // velocity
	Eigen::Vector3d force;

	std::vector<Particle*>		 particles;
	std::vector<Eigen::Vector3d> weightGradients; // per particle
};
