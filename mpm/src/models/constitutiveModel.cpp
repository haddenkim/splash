#include "constitutiveModel.h"
#include <cmath>

using namespace Eigen;

ConstitutiveModel::ConstitutiveModel(double thetaC,
									 double thetaS,
									 double xi,
									 double E0,
									 double nu)
	: thetaC_(thetaC)
	, thetaS_(thetaS)
	, xi_(xi)
{
	// compute and store initial mu and lambda (eq 47)
	mu0_	 = E0 / (2 * (1 + nu));
	lambda0_ = E0 * nu / ((1 + nu) * (1 - 2 * nu));
}

void ConstitutiveModel::updateDeformDecomp(Eigen::Matrix3d&		  F_E,
										   Eigen::Matrix3d&		  R_E,
										   Eigen::Matrix3d&		  F_P,
										   double&				  J_P,
										   const Eigen::Matrix3d& velGradient,
										   const double&		  timestep)
{
	// compute updated elastic deformation gradient F_E_Tilda (eq 181 and paragraph above eq 80)
	Matrix3d F_E_Tilda = (Matrix3d::Identity() + timestep * velGradient) * F_E;

	// compute updated deformation gradient F (eq 80)
	Matrix3d F = F_E_Tilda * F_P;

	// SVD of F_E_Tilda (eq 83)
	JacobiSVD<Matrix3d> svd(F_E_Tilda, ComputeFullU | ComputeFullV);
	Matrix3d			U   = svd.matrixU();		// U
	Vector3d			Sig = svd.singularValues(); // Σ
	Matrix3d			V   = svd.matrixV();		// V

	// clamp singular values (eq 82)
	for (int i = 0; i < 3; i++) {
		Sig(i) = std::max(Sig(i), 1.0 - thetaC_);
		Sig(i) = std::min(Sig(i), 1.0 + thetaS_);
	}

	// update part F_E (eq 84) and R_E (paragraph under eq 45)
	F_E = U * Sig.asDiagonal() * V.transpose();
	R_E = U * V.transpose();

	// compute part F_P (eq 86) and its determinant J_P
	F_P = F_E.inverse() * F;
	J_P = F_P.determinant();
}

Eigen::Matrix3d ConstitutiveModel::computeFirstPiolaKirchoff(const Eigen::Matrix3d& F_E,
															 const Eigen::Matrix3d& R_E,
															 const double&			J_P)
{
	// compute current mu and lambda (eq 87)
	double exp	= std::exp(xi_ * (1.0 - J_P));
	double mu	 = mu0_ * exp;
	double lambda = lambda0_ * exp;

	// compute determinant of elastic deformation gradient J_E
	double J_E = F_E.determinant();

	// compute first Piola-Kirchoff Stress P (eq 52)
	Matrix3d P_hat = 2 * mu * (F_E - R_E) + (lambda * (J_E - 1) * J_E) * F_E.transpose();
}

Eigen::Matrix3d ConstitutiveModel::computeVolCauchyStress(const double&			 vol0,
														  const Eigen::Matrix3d& F_E,
														  const Eigen::Matrix3d& R_E,
														  const double&			 J_P)
{
	// compute current mu and lambda (eq 87)
	double exp	= std::exp(xi_ * (1.0 - J_P));
	double mu	 = mu0_ * exp;
	double lambda = lambda0_ * exp;

	// compute determinant of elastic deformation gradient J_E
	double J_E = F_E.determinant();

	// compute the per part constant contribution to nodal elastic force V_p * P_p * (F_p)^T (eq 189)
	// aka current volume * cauchy stress V_p * σ_p (eq 190)
	Matrix3d PFT = 2 * mu * (F_E - R_E) * F_E.transpose() + (lambda * (J_E - 1) * J_E) * Matrix3d::Identity();

	return vol0 * PFT;
}
