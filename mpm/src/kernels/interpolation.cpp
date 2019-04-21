#include "interpolation.h"

Interpolation::Interpolation()
{
}

Interpolation::Interpolation(Eigen::Vector3d position, double dx)
{
	compute(position, dx);
}

void Interpolation::compute(Eigen::Vector3d position, double dx)
{
	// TODO: consider subclassing this for alternative interpolations (ex. linear, cubic)
	// For now all functions here implement Quadradic B-spline interpolation (eq 123)

	// part position in grid frame
	Eigen::Vector3d partGridPos = position ;

	// lowest node position in grid frame of part's kernel
	node0 = (partGridPos.array() - 0.5).cast<int>();

	// vector from start node to part in grid frame (aka particle position in kernel frame)
	vecI0P = partGridPos - node0.cast<double>();

	// compute weight components w_aip (eq 123)
	Eigen::Vector3d wComp[3];
	wComp[0] = 0.50 * (1.5 - vecI0P.array()).square(); // start node
	wComp[1] = 0.75 - (vecI0P.array() - 1.0).square(); // middle node
	wComp[2] = 0.50 * (vecI0P.array() - 0.5).square(); // end node

	// compute weight gradient components ∇w_aip / h
	Eigen::Vector3d wGrad_HComps[3];
	wGrad_HComps[0] = (vecI0P.array() - 1.5) ;			// start node
	wGrad_HComps[1] = (-2.0 * (vecI0P.array() - 1.0)) ; // middle node
	wGrad_HComps[2] = (vecI0P.array() - 0.5);			// end node

	// compute per node weight and weight gradient ∇w_aip
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				w[i][j][k]		 = wComp[i].x() * wComp[j].y() * wComp[k].z();
				wGrad_H[i][j][k] = Eigen::Vector3d(wGrad_HComps[i].x() * wComp[j].y() * wComp[k].z(),
												   wComp[i].x() * wGrad_HComps[j].y() * wComp[k].z(),
												   wComp[i].x() * wComp[j].y() * wGrad_HComps[k].z());
			}
		}
	}
}

double Interpolation::weight(int i, int j, int k) const
{
	// compute weight N_ip (eq. 121)
	return w[i][j][k];
}

Eigen::Vector3d Interpolation::weightGradient(int i, int j, int k) const
{
	// compute weight gradient ∇N_ip (eq. after 124)'
	return wGrad_H[i][j][k];
}

Eigen::Vector3d Interpolation::vecPI(int i, int j, int k) const
{
	// compute vector from part to node in grid frame (x_i - x_p) / dx
	return Eigen::Vector3d(i, j, k) - vecI0P;
}

double Interpolation::DInverseScalar(double dx)
{
	// compute common inertia-like tensor inverse (D_p)^-1 (paragraph after eq. 176)
	return 4.0 ; // (1/4 * (∆x)^2 * I)^-1
}
