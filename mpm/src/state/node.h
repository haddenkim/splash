#pragma once
#include "settings/constants.h"
#include <Eigen/Core>
#include <array>

struct Particle;

struct Node {
	Eigen::Vector3i pos;
	Eigen::Vector3d vel;
	Eigen::Vector3d force;
	double			mass;
};

struct NodeBlock {
	int nodeBegin;

	int partBegin;
	int partEnd;
};

using NodeSet		= std::array<u_int32_t, SET_NUM_BLOCKS>;
using NodeNeighbors = std::array<Node*, NEIGHBOR_NUM_NODES>;