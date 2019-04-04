#pragma once
#include <Eigen/Core>
#include <vector>

struct Particle;

struct Node {
	// for rendering
	Eigen::Vector3d vel; // velocity
	Eigen::Vector3d force;

	// time integration bookkeeping
	double						 mass;			  // mass
	std::vector<Particle*>		 particles;		  // nearby particles that contibute to this node
	std::vector<Eigen::Vector3d> weightGradients; // per particle
};
