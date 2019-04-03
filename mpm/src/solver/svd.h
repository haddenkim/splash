#pragma once
#include <Eigen/Dense>

using namespace Eigen;

static void svd(const Matrix3d& m,
				Matrix3d&		u,
				Matrix3d&		sig,
				Matrix3d&		v)
{
	JacobiSVD<Matrix3d> svd(m, ComputeFullU | ComputeFullV);
	u				= svd.matrixU();		// U
	Vector3d sigVec = svd.singularValues(); // Σ
	v				= svd.matrixV();		// V

	for (int i = 0; i < 3; i++) {
		if (sigVec[i] < 0) {
			sigVec[i] = -sigVec[i];
			u.row(i) *= -1.0;
			// u[i] *= -1.0;
		}
	}

	sig = sigVec.asDiagonal();
}
