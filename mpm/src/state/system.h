#pragma once

#include "state/node.h"
#include "state/particle.h"
#include "state/particleNodeLink.h"
#include <Eigen/Core>
#include <vector>

class System {
public:
	System(Eigen::Vector3d cellSize, Eigen::Vector3d worldSize);

	void addParticle(double m, double vol, double x, double y, double z, double vx, double vy, double vz);
	void clearParticles();

	Node* getNodeAt(int x, int y, int z);

	// particle
	std::vector<Particle> particles_;

	// grid z > y > x
	std::vector<Node> nodes_;
	int				  activeNodes_;

	// particle grid links
	// TODO: find alternative data structure for 2 way (node + particle) iterators
	std::vector<std::vector<ParticleNodeLink>> linksByNode_;	 // list of each Node's particle links. order matching node list
	std::vector<std::vector<ParticleNodeLink>> linksByParticle_; // list of each particle's links. order matching particle list

	// dimensions
	Eigen::Vector3d worldOrigin_; // origin in world frame
	Eigen::Vector3d cellSize_;	// size of an individual cell (node to node spacing) in world frame
	Eigen::Vector3i gridSize_;	// number of nodes in grid
	Eigen::Vector3d worldSize_;   // size of grid in world frame

	double			kernelRadius_; // kernel radius in world frame
	Eigen::Vector3i kernelSize_;   // size of kernel in grid frame
	Eigen::Vector3i kernelOffset_; // maps particle position to smallest node in kernel

private:
	void initGrid();
	void computeKernelSize();
};