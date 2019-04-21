#include "sandModel.h"

using namespace Eigen;

SandModel::SandModel()
{
}

double SandModel::computePotentialEnergy(const Eigen::Matrix3d& F_E,
										 const Eigen::Matrix3d& R_E,
										 const Eigen::Matrix3d& F_P,
										 double					vol0) const
{
	return 0;
}

void SandModel::updateDeformDecomp(Eigen::Matrix3d&		  F_E,
								   Eigen::Matrix3d&		  R_E,
								   Eigen::Matrix3d&		  F_P,
								   double&				  J_P,
								   const Eigen::Matrix3d& velGradient,
								   const double&		  timestep) const
{
	double alpha;

	// compute updated elastic deformation gradient F̂_E (D-P eq 16)
	Matrix3d F_E_hat = (Matrix3d::Identity() + timestep * velGradient) * F_E;

	project(F_E_hat, alpha);
}

Eigen::Matrix3d SandModel::computeVolCauchyStress(const double&			 vol0,
												  const Eigen::Matrix3d& F_E,
												  const Eigen::Matrix3d& R_E,
												  const double&			 J_P) const
{
	JacobiSVD<Matrix3d> svd(F_E, ComputeFullU | ComputeFullV);
	Matrix3d			U   = svd.matrixU();		// U
	Vector3d			Sig = svd.singularValues(); // Σ
	Matrix3d			V   = svd.matrixV();		// V

	// lnΣ defined as logarithm of diagonal entries (para above D-P eq 26)
	Matrix3d lnSig  = Sig.array().log().matrix().asDiagonal();
	Matrix3d invSig = Sig.cwiseInverse().asDiagonal();

	// (D-P eq 26)
	Matrix3d P = U * (2 * mu0_ * invSig * lnSig + lambda0_ * lnSig.trace() * invSig) * V.transpose();

	return vol0 * P * F_E.transpose();
}

// computes First Piola Kirchoff Differential δP
Eigen::Matrix3d SandModel::computeFirstPiolaKirchoffDifferential(const Eigen::Matrix3d& differentialF_E,
																 const Eigen::Matrix3d& F_E,
																 const double&			J_P) const
{
	assert(!"not yet implemented");
	return Matrix3d::Identity();
}

void SandModel::project(Eigen::Matrix3d& F_E, double& alpha) const
{
}
