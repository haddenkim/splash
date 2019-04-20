#pragma once
#include "models/constitutiveModel.h"
#include "state/node.h"
#include "state/particle.h"
#include <vector>

class System {
public:
	System();

	void reset(int size);
	void addCube(int partCount, Eigen::Vector3d center, Eigen::Vector3d velocity, Eigen::RowVector3d color);

	unsigned getNodeIndex(unsigned x, unsigned y, unsigned z) const;
	Node*	getNode(int x, int y, int z);
	Node*	getNode(Eigen::Vector3i pos);

	bool isInBounds(double x, double y, double z) const;
	bool isInBounds(Eigen::Vector3d pos) const;

	void sortParticles();

	// grid dimensions
	int	gridSize_;  //
	int	nodeCountX; // number of nodes in each dimension
	double dx_ = 1;	// ∆x distance between nodes. Simulation is performed entirely in "grid space", so dx is always 1

	// array of particle structs
	std::vector<Particle> particles_;

	// x , y , z order of grid nodes
	std::vector<Node> nodes_;

	// TODO store rigid bodies

	// TODO: create more sophisticated boundary conditions
	// boundaries
	int boundaryStart_;
	int boundaryEnd_;

	// particle model
	ConstitutiveModel constitutiveModel_;

private:
	//helpers
	void setupGrid(int size);

	int nodeCountXY_; // number of nodes in the X Y plane
};