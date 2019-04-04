#pragma once
#include <Eigen/Dense>

struct Interpolation {
	Interpolation(Eigen::Vector3d position, double dx)
	{
		// TODO: consider subclassing this for alternative interpolations (ex. linear, cubic)
		// For now all functions here implement Quadradic B-spline interpolation (eq 123)

		// part position in grid frame
		Eigen::Vector3d partGridPos = position / dx;

		// lowest node position in grid frame of part's kernel
		node0 = (partGridPos.array() - 0.5).cast<int>();

		// vector from start node to part in grid frame (aka particle position in kernel frame)
		vecI0P = partGridPos - node0.cast<double>();

		// compute weight components w_aip (eq 123)
		w[0] = 0.50 * (1.5 - vecI0P.array()).square(); // start node
		w[1] = 0.75 - (vecI0P.array() - 1.0).square(); // middle node
		w[2] = 0.50 * (vecI0P.array() - 0.5).square(); // end node

		// compute weight gradient components ∇w_aip / h
		wGrad_H[0] = (vecI0P.array() - 1.5) / dx;		  // start node
		wGrad_H[1] = (-2.0 * (vecI0P.array() - 1.0)) / dx; // middle node
		wGrad_H[2] = (vecI0P.array() - 0.5) / dx;		  // end node
	}

	double weight(int i, int j, int k)
	{
		// TODO consider storing these 9 values at P2G to avoid recompute during G2P

		// compute weight N_ip (eq. 121)
		return w[i].x() * w[j].y() * w[k].z();
	}

	Eigen::Vector3d weightGradient(int i, int j, int k)
	{
		// TODO consider storing these 9 values at P2G to avoid recompute during G2P

		// compute weight gradient ∇N_ip (eq. after 124)'
		return Eigen::Vector3d(wGrad_H[i].x() * w[j].y() * w[k].z(),
							   w[i].x() * wGrad_H[j].y() * w[k].z(),
							   w[i].x() * w[j].y() * wGrad_H[k].z());
	}

	Eigen::Vector3d vecPI(int i, int j, int k)
	{
		// compute vector from part to node in grid frame (x_i - x_p) / dx
		return Eigen::Vector3d(i, j, k) - vecI0P;
	}

	static double DInverseScalar(double dx)
	{
		// compute common inertia-like tensor inverse (D_p)^-1 (paragraph after eq. 176)
		return 4.0 / dx / dx; // (1/4 * (∆x)^2 * I)^-1
	}

	// data
	Eigen::Vector3i node0;  // node position of 1st node in grid frame
	Eigen::Vector3d vecI0P; // vector from start node (I_0) to particle in grid frame

	Eigen::Vector3d w[3];		// w_aip
	Eigen::Vector3d wGrad_H[3]; // ∇w_aip / dx
};
