#pragma once
#include "state/node.h"
#include "state/particle.h"
#include <array>
#include <omp.h>

#define BLOCK_SIZE_X 4
#define BLOCK_SIZE_Y 4
#define BLOCK_SIZE_Z 4

struct NodeBlock {
	std::array<Node, BLOCK_SIZE_X * BLOCK_SIZE_Y * BLOCK_SIZE_Z> nodes;
	std::vector<Particle*>										 ownedParticles;

	// 3x3x3 in x->y->z order of neighbors, including self
	std::array<NodeBlock*, 27> neighbors;

	omp_lock_t lock;
};
