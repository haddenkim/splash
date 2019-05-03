#pragma once

#include "models/constitutiveModel.h"
#include "settings/constants.h"
#include "state/node.h"
#include "state/particle.h"

#include <Eigen/Core>
#include <array>
#include <omp.h>
#include <vector>

struct System {

	// node data
	std::array<Node, WORLD_NUM_NODES>		   nodes;
	std::array<NodeBlock, WORLD_NUM_BLOCKS>	blocks;
	std::array<NodeSet, WORLD_NUM_SETS>		   sets;
	std::array<NodeNeighbors, WORLD_NUM_NODES> neighbors;

	// particle data
	int							 partCount; // number of particles in ssytem (for convenience)
	bool						 partDataSorted;
	std::vector<ParticleBase>	particles; // sort indices
	std::vector<Eigen::Vector3d> partPos;   // position

	// particle momentum
	std::vector<double>			 partMass; //  mass
	std::vector<Eigen::Vector3d> partVel;  // velocity
	std::vector<Eigen::Matrix3d> partB;	// affine momentum

	// particle deformation
	std::vector<ConstitutiveModel*> partModel;

	// particle rendering
	std::vector<Eigen::Vector3d> partColor;

	// temps
	std::vector<Eigen::Matrix3d> partVelGrad;

	// TODO: create more sophisticated boundary conditions
	// boundaries
	int boundaryStart;
	int boundaryEnd;

	// reorder buffer
	std::vector<double> partReorderBuffer;
};
