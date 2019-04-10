#pragma once
#include <Eigen/Dense>

struct Interpolation {
	Interpolation();
	Interpolation(Eigen::Vector3d position, double dx);

	// compute the underlying kernel data
	void compute(Eigen::Vector3d position, double dx);

	// retrieve 
	double weight(int i, int j, int k);

	Eigen::Vector3d weightGradient(int i, int j, int k);

	Eigen::Vector3d vecPI(int i, int j, int k);

	static double DInverseScalar(double dx);

	// data
	Eigen::Vector3i node0;  // node position of 1st node in grid frame
	Eigen::Vector3d vecI0P; // vector from start node (I_0) to particle in grid frame

	Eigen::Vector3d w[3];		// w_aip
	Eigen::Vector3d wGrad_H[3]; // ∇w_aip / dx
};
