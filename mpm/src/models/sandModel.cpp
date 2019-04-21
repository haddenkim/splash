#include "sandModel.h"

using namespace Eigen;

SandModel::SandModel()
{
}

void SandModel::updateDeformDecomp(Eigen::Matrix3d&		  F_E,
								   Eigen::Matrix3d&		  R_E,
								   Eigen::Matrix3d&		  F_P,
								   double&				  J_P,
								   const Eigen::Matrix3d& velGradient,
								   const double&		  timestep) const
{
}

Eigen::Matrix3d SandModel::computeVolCauchyStress(const double&			 vol0,
												  const Eigen::Matrix3d& F_E,
												  const Eigen::Matrix3d& R_E,
												  const double&			 J_P) const
{
	return Matrix3d::Identity();
}

// computes First Piola Kirchoff Differential δP
Eigen::Matrix3d SandModel::computeFirstPiolaKirchoffDifferential(const Eigen::Matrix3d& differentialF_E,
																 const Eigen::Matrix3d& F_E,
																 const double&			J_P) const
{
	return Matrix3d::Identity();
}
