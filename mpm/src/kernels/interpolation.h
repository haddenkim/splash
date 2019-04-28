#pragma once
#include <Eigen/Dense>

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

	// data
	Eigen::Vector3i node0;  // node position of 1st node in grid frame
	Eigen::Vector3d vecI0P; // vector from start node (I_0) to particle in grid frame

	double			w[3][3][3];		  // w_aip
	Eigen::Vector3d wGrad_H[3][3][3]; // ∇w_aip
};
