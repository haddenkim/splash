#include "models/snowModel.h"
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

// Stomakhin Snow model and default parameters (stom Table 2)
// lowered E from 1.4e5 to 1e4 to induce more fracturing
const double SnowModel::thetaC_ = 2.5e-2; // θ_c critical compression
const double SnowModel::thetaS_ = 7.5e-3; // θ_s critical stretch
const double SnowModel::xi_		= 10.0;   // ξ hardening coefficient
const double SnowModel::E0_		= 1e4;	// E_0 initial Young's Modulus
const double SnowModel::nu_		= 0.2;	// ν Poisson ratio

// initial mu and lambda (eq 47)
const double SnowModel::mu0_	 = E0_ / (2 * (1 + nu_));					// μ shear modulus
const double SnowModel::lambda0_ = E0_ * nu_ / ((1 + nu_) * (1 - 2 * nu_)); // λ Lame's first parameter

SnowModel::SnowModel(double vol0)
	: vol0_(vol0)
	, F_E_(Matrix3d::Identity())
	, R_E_(Matrix3d::Identity())
	, F_P_(Matrix3d::Identity())
{
}

void SnowModel::updateDeformation(const Eigen::Matrix3d& velGradient, const double timestep)
{
	// compute updated elastic deformation gradient F_E_Tilda (eq 181 and paragraph above eq 80)
	Matrix3d F_E_Tilda = (Matrix3d::Identity() + timestep * velGradient) * F_E_;

	// compute updated deformation gradient F (eq 80)
	Matrix3d F = F_E_Tilda * F_P_;

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

	// update part F_E_ (eq 84) and R_E_ (paragraph under eq 45)
	F_E_ = U * Sig.asDiagonal() * V.transpose();
	R_E_ = U * V.transpose();

	// compute part F_P_ (eq 86)
	F_P_ = F_E_.inverse() * F;
}

double SnowModel::computePotentialEnergy() const
{
	double J_E = F_E_.determinant();
	double J_P = F_P_.determinant();

	// compute current mu and lambda (eq 87)
	double mu;
	double lambda;
	computeMuLambda(mu, lambda, J_P);

	// (stom eq  1)
	return mu * (F_E_ - R_E_).squaredNorm() + 0.5 * lambda * (J_E - 1) * (J_E - 1);
}

Eigen::Matrix3d SnowModel::computeVolCauchyStress() const
{
	double J_E = F_E_.determinant();
	double J_P = F_P_.determinant();

	// compute current mu and lambda (eq 87)
	double mu;
	double lambda;
	computeMuLambda(mu, lambda, J_P);

	// compute the per part constant contribution to nodal elastic force V_p * P_p * (F_p)^T (eq 189)
	// aka current volume * cauchy stress V_p * σ_p (eq 190)
	Matrix3d PFT = 2 * mu * (F_E_ - R_E_) * F_E_.transpose() + (lambda * (J_E - 1) * J_E) * Matrix3d::Identity();

	return vol0_ * PFT;
}

void SnowModel::computeMuLambda(double& mu, double& lambda, const double& J_P) const
{
	// compute current mu and lambda (eq 87)
	double exp = std::exp(xi_ * (1.0 - J_P));
	mu		   = mu0_ * exp;
	lambda	 = lambda0_ * exp;
}

// for rendering
double SnowModel::getRenderElastic() const
{
	return F_E_.determinant();
}

double SnowModel::getRenderPlastic() const
{
	return F_P_.determinant();
}

// for GUI
std::string SnowModel::getGui() const
{
}