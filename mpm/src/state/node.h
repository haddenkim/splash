#pragma once
#include <Eigen/Core>
#include <omp.h>
#include <set>
#include <vector>

struct Particle;

struct Node {
	// position in grid space
	int x;
	int y;
	int z;

	// time dependent
	Eigen::Vector3d vel; // velocity
	Eigen::Vector3d force;

	// time integration bookkeeping
	double mass;

	// for node first P2G
	std::set<int> ownedParticles;

	// only used by implicit solvers
	std::vector<int>			 particles;		  // nearby particles that contibute to this node
	std::vector<Eigen::Vector3d> weightGradients; // per particle
	int							 activeNodeIndex; // index in the active node list

	// only used by openMP solver
	int		   approxParts; // number of particles in node's range. intentionally allowed race condition
	omp_lock_t lock;
};
