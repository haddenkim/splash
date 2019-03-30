#pragma once

#include "state/node.h"
#include "state/particle.h"

struct ParticleNodeLink {
	ParticleNodeLink(Particle* particle, Node* node, double weight, Eigen::Vector3d weightGradient)
		: particle(particle)
		, node(node)
		, weight(weight)
		, weightGradient(weightGradient)
	{
	}

	Particle*		particle;
	Node*			node;
	double			weight;
	Eigen::Vector3d weightGradient;
};
