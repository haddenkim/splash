#pragma once

#include <Eigen/Core>
#include <vector>

struct Node {
	Node(int index, Eigen::Vector3d position)
		: gridIndex(index)
		, position(position)
	{
	}

	int				gridIndex; // global index in the system's node list (1D)
	Eigen::Vector3d position;  // position in world frame

	double			mass;
	Eigen::Vector3d momentum;
	Eigen::Vector3d velocity;

	// grid timestep
	bool active;
	Eigen::Vector3d force;
};