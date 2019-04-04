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
	static const int n_		  = 80;
	static const int gridSize = n_ + 1;   // number of nodes in each dimension
	const double	 dx		  = 1.0 / (gridSize - 1); // ∆x distance between nodes

	// array of particle structs
	std::vector<Particle> particles_;

	// x , y , z order of grid nodes
	Node nodes_[gridSize][gridSize][gridSize];

	// boundaries
	double boundary = 0.05;
};