#pragma once
#include "state/node.h"
#include "state/particle.h"
#include <vector>

class System {
public:
	System();

	void clear();
	void addCube(Eigen::Vector3d center, Eigen::Vector3d velocity, Eigen::RowVector3d color);

	// grid dimensions
	static const int n = 80; // Grid resolution (cells)
	static const int gridSize = 81;		 // number of nodes in each dimension
	double			 dx		  = 1.0 / n; // distance between nodes

	// array of particle structs
	std::vector<Particle> particles_;
	// x , y , z order of grid ndoes
	Node			nodes_[gridSize][gridSize][gridSize];





	// boundaries
	double boundary = 0.05;
};