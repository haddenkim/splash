#pragma once
#include "models/modelType.h"
#include "settings/constants.h"
#include "settings/systemStart.h"
#include "state/node.h"
#include "state/particle.h"
#include "state/shape.h"
#include <array>
#include <vector>

class ConstitutiveModel;

class System {
public:
	void restart(SystemStart start);

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
	// int		   blockCount() const;
	// unsigned   getBlockIndex(unsigned x, unsigned y, unsigned z) const;
	// NodeBlock& getBlock(int x, int y, int z);
	// NodeBlock& getBlock(Eigen::Vector3i pos);
	// NodeBlock& getBlock(Eigen::Vector3d pos);

	bool isInBounds(double x, double y, double z) const;
	bool isInBounds(Eigen::Vector3d pos) const;

	void sortParticles();

	// TODO: create more sophisticated boundary conditions
	// boundaries
	int boundaryStart_;
	int boundaryEnd_;

	// particle model(s)
	std::vector<ConstitutiveModel*> constitutiveModels;

private:
	//helpers
	void setupGrid();
	void addShapes(std::vector<Shape> shapes);
	void addPart(ModelType type, Eigen::Vector3d pos, Eigen::Vector3d velocity, Eigen::RowVector3d color);

	// sim data
	std::vector<Particle> particles_;
	std::vector<Node>	 nodes_;

	// TODO store rigid bodies
};