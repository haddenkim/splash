#pragma once
#include "models/constitutiveModel.h"
#include "state/node.h"
#include "state/particle.h"
#include <vector>

class System {
public:
	System();

	void clear();
	void addCube(int partCount, Eigen::Vector3d center, Eigen::Vector3d velocity, Eigen::RowVector3d color);

	// grid dimensions
	static const int gridSize_ = 51;					// number of nodes in each dimension
	const double	 dx_	   = 1.0 / (gridSize_ - 1); // ∆x distance between nodes

	// array of particle structs
	std::vector<Particle> particles_;

	// x , y , z order of grid nodes
	Node nodes_[gridSize_][gridSize_][gridSize_];

	// boundaries
	double boundary_ = 0.05;

	// particle modes
	ConstitutiveModel constitutiveModel_;

private:
	//helpers
	void setupGrid();
};