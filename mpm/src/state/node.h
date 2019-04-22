#pragma once
#include <Eigen/Core>
#include <array>
#include <omp.h>
#include <set>
#include <vector>

struct Particle;

struct Node {
	// position in grid space
	int x;
	int y;
	int z;

	// 3x3x3 block in x->y->z order of neighbors, including self
	std::array<Node*, 27> neighbors;

	// time dependent
	Eigen::Vector3d vel; // velocity
	Eigen::Vector3d force;
	double			mass;

	// for node first P2G
	std::set<Particle*> ownedParticles;

	// only used by implicit solvers
	Eigen::Vector3d differentialU; // δu aka δvelocity

	// only used by openMP solver
	int		   approxParts; // number of particles in node's range. intentionally allowed race condition
	omp_lock_t lock;
};
