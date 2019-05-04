#pragma once
#include "settings/constants.h"
#include "state/particle.h"
#include <Eigen/Core>

using PartWeights		  = double[KERNEL_NUM_NODES_X][KERNEL_NUM_NODES_X][KERNEL_NUM_NODES_X];			 // w_aip
using PartWeightGradients = Eigen::Vector3d[KERNEL_NUM_NODES_X][KERNEL_NUM_NODES_X][KERNEL_NUM_NODES_X]; // ∇w_aip

struct Interpolation {
	Interpolation();
	Interpolation(Eigen::Vector3d position);

	// compute the underlying kernel data
	void compute(Eigen::Vector3d position);

	// retrieve
	double weight(int i, int j, int k) const;

	Eigen::Vector3d weightGradient(int i, int j, int k) const;

	Eigen::Vector3d vecPI(int i, int j, int k) const;

	static double DInverseScalar();

	static void computeWeights(PartWeights& weights, PartWeightGradients& gradients, const Eigen::Vector3d pos);

	// data
	Eigen::Vector3i node0;  // node position of 1st node in grid frame
	Eigen::Vector3d vecI0P; // vector from start node (I_0) to particle in grid frame

	double			w[3][3][3];		  // w_aip
	Eigen::Vector3d wGrad[3][3][3]; // ∇w_aip
};
