#pragma once
#include <Eigen/Dense>

// some tensor helpers
double doubleContraction(Eigen::Matrix3d A, Eigen::Matrix3d B)
{
	return (A.array() * B.array()).sum();
}