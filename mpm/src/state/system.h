#pragma once
#include "models/constitutiveModel.h"
#include "state/node.h"
#include "state/nodeBlock.h"
#include "state/particle.h"
#include <vector>

class System {
public:
	System();

	void reset(int size);
	void addPart(Eigen::Vector3d pos, Eigen::Vector3d velocity, Eigen::RowVector3d color);

	// particle getters
	int				partCount() const;
	Particle&		getPart(int pi);
	const Particle& getPart(int pi) const;

	// node getters
	int			nodeCount() const;
	unsigned	getNodeIndex(unsigned x, unsigned y, unsigned z) const;
	Node&		getNode(int ni);
	const Node& getNode(int ni) const;
	Node&		getNode(int x, int y, int z);
	Node&		getNode(Eigen::Vector3i pos);
	Node&		getNode(Eigen::Vector3d pos);

	// block getters (in grid space)
	int		   blockCount() const;
	unsigned   getBlockIndex(unsigned x, unsigned y, unsigned z) const;
	NodeBlock& getBlock(int x, int y, int z);
	NodeBlock& getBlock(Eigen::Vector3i pos);
	NodeBlock& getBlock(Eigen::Vector3d pos);

	bool isInBounds(double x, double y, double z) const;
	bool isInBounds(Eigen::Vector3d pos) const;

	void sortParticles();

	// grid dimensions
	int	gridSize_;  //
	int	nodeCountX; // number of nodes in each dimension
	double dx_ = 1;	// ∆x distance between nodes. Simulation is performed entirely in "grid space", so dx is always 1

	// TODO: create more sophisticated boundary conditions
	// boundaries
	int boundaryStart_;
	int boundaryEnd_;

	// particle model(s)
	std::vector<ConstitutiveModel*> constitutiveModels;

private:
	//helpers
	void setupGrid(int size);

	// sim data
	std::vector<Particle>  particles_;
	std::vector<Node>	  nodes_;
	std::vector<NodeBlock> blocks_;

	// TODO store rigid bodies

	int				nodeCountXY_; // number of nodes in the X Y plane
	Eigen::Vector3i blockCount_;  // number of blocks in X, Y, Z directions
};