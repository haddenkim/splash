#pragma once

#include "state/node.h"
#include "state/particle.h"

struct ParticleNodeLink {
	ParticleNodeLink(int particleIndex, int nodeIndex, double weight, Eigen::Vector3d weightGradient)
		: particleIndex(particleIndex)
		, nodeIndex(nodeIndex)
		, weight(weight)
		, weightGradient(weightGradient)
	{
	}

	int				particleIndex;
	int				nodeIndex;
	double			weight;
	Eigen::Vector3d weightGradient;
};
