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
	std::vector<Eigen::Matrix3d> partB;	// affine state

	// particle deformation
	std::vector<ConstitutiveModel*> partModel;
	std::vector<double>				partVol0; // initial volume
	std::vector<Eigen::Matrix3d>	partF_E;  // elastic part of deformation gradient F
	std::vector<Eigen::Matrix3d>	partF_P;  // plastic part of deformation gradient F
	std::vector<Eigen::Matrix3d>	partR_E;  // rotational decomposition of F_E
	std::vector<double>				partJ_P;  // determinant of the plastic deformation gradient

	// particle rendering
	std::vector<Eigen::Vector3d> partColor;

	// temps
	std::vector<Eigen::Matrix3d> partVelGrad;

	// TODO: create more sophisticated boundary conditions
	// boundaries
	int boundaryStart;
	int boundaryEnd;

	// particle model(s)
	std::vector<ConstitutiveModel*> constitutiveModels;

	// reorder buffer
	std::vector<double> partReorderBuffer;
};
