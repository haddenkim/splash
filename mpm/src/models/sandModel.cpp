#include "models/sandModel.h"
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

// (Klar table 3)
const double SandModel::E_		= 3.537e5;
const double SandModel::nu_		= 0.3;
const double SandModel::mu_		= E_ / (2 * (1 + nu_));
const double SandModel::lambda_ = E_ * nu_ / ((1 + nu_) * (1 - 2 * nu_));

// Note: Klar provides parameters in degrees, so converted here to radians
const double SandModel::h0_			   = 35 * M_PI / 180;
const double SandModel::h1_			   = 9 * M_PI / 180;
const double SandModel::h2_			   = 0.2 * M_PI / 180;
const double SandModel::h3_			   = 10 * M_PI / 180;
const double SandModel::sqrtTwoThirds_ = sqrt(2.0 / 3.0);

SandModel::SandModel(double vol0)
	: vol0_(vol0)
	, F_E_(Matrix3d::Identity())
	, q_(1)
	, alpha_(computeHardening(q_))
{
}

void SandModel::updateDeformation(const Eigen::Matrix3d& velGradient, const double timestep)
{

	// compute elastic deformation gradient, before plasticity F̂_E (Klar eq 16)
	Matrix3d F_E_hat = (Matrix3d::Identity() + timestep * velGradient) * F_E_;

	// SVD of F̂_E
	JacobiSVD<Matrix3d> svd(F_E_hat, ComputeFullU | ComputeFullV);
	Matrix3d			U   = svd.matrixU();		// U
	Vector3d			Sig = svd.singularValues(); // Σ
	Matrix3d			V   = svd.matrixV();		// V

	// compute T (stored in Sig) and δq (stored in alpha)
	projectToYieldSurface(Sig, alpha_);

	// update F_E
	F_E_ = U * Sig.asDiagonal() * V.transpose();

	// update hardening (Klar eq 29 - 31)
	q_ += alpha_;
	alpha_ = computeHardening(q_);
}

double SandModel::computePotentialEnergy() const
{
	// SVD of F_E
	JacobiSVD<Matrix3d> svd(F_E_, ComputeFullU | ComputeFullV);
	Matrix3d			U   = svd.matrixU();		// U
	Vector3d			Sig = svd.singularValues(); // Σ
	Matrix3d			V   = svd.matrixV();		// V

	// Klar para above eq 26
	Vector3d lnSig	= Sig.array().log();			  // lnΣ
	double   trlnSig  = lnSig.sum();				  // tr(lnΣ)
	double   trln2Sig = lnSig.array().square().sum(); // tr((lnΣ)^2)

	return mu_ * trln2Sig + 0.5 * lambda_ * trlnSig * trlnSig;
}

Eigen::Matrix3d SandModel::computeVolCauchyStress() const
{
	// SVD of F_E
	JacobiSVD<Matrix3d> svd(F_E_, ComputeFullU | ComputeFullV);
	Matrix3d			U   = svd.matrixU();		// U
	Vector3d			Sig = svd.singularValues(); // Σ
	Matrix3d			V   = svd.matrixV();		// V

	// Klar para above eq 26
	Matrix3d lnSig  = Sig.array().log().matrix().asDiagonal();	 // lnΣ
	Matrix3d invSig = Sig.array().inverse().matrix().asDiagonal(); // Σ^-1

	// Klar eq 26
	Matrix3d P = U * (2 * mu_ * invSig * lnSig + lambda_ * lnSig.trace() * invSig) * V.transpose();

	return vol0_ * P * F_E_.transpose();
}

void SandModel::projectToYieldSurface(Eigen::Vector3d& Sig, double& alpha) const
{
	// compute ϵ and ϵ̂  (Klar eq 27)
	Vector3d e		   = Sig.array().log();
	double   eTrace	= e.array().sum();
	Vector3d e_hat	 = e - eTrace / 3 * Vector3d::Ones();
	double   e_hatNorm = e_hat.norm();

	// Case II
	if (e_hatNorm == 0 || eTrace > 0) {
		Sig   = Vector3d::Ones();
		alpha = e_hatNorm;
		return;
	}

	// compute amount of plastic deformation δγ (Klar eq 27)
	// small optimization: store δγ in alpha
	alpha = e_hatNorm + ((3 * lambda_ * 0.5 / mu_) + 1) * eTrace * alpha;

	// Case I
	if (alpha <= 0) { // δγ <= 0
		alpha = 0;
		return;
	}

	// Case III
	else {
		// (Klar eq 28) note δγ is in alpha
		Vector3d H = e - alpha * e_hat / e_hatNorm;

		// e^H
		Sig = H.array().exp();
		// alpha already = δγ
		return;
	}
}

double SandModel::computeHardening(double q) const
{
	// (Klar eq 30)
	double phi = h0_ + (h1_ * q_ - h3_) * exp(-h2_ * q_);
	// phi			  = 30 * M_PI / 180; // debug
	double sinPhi = sin(phi);

	// (Klar eq 31)
	return sqrtTwoThirds_ * 2 * sinPhi / (3 - sinPhi);
}

// for rendering
double SandModel::getRenderElastic() const
{
	return (F_E_.determinant() - 1) * 100;
}

double SandModel::getRenderPlastic() const
{
	return (q_ - 1) / 5;
}

// for GUI
std::string SandModel::getGui() const
{
	std::string ret;

	// deformation gradient
	ret = "F_E\n";
	ret += std::to_string(F_E_(0, 0)) + "\t" + std::to_string(F_E_(0, 1)) + "\t" + std::to_string(F_E_(0, 2)) + "\n";
	ret += std::to_string(F_E_(1, 0)) + "\t" + std::to_string(F_E_(1, 1)) + "\t" + std::to_string(F_E_(1, 2)) + "\n";
	ret += std::to_string(F_E_(2, 0)) + "\t" + std::to_string(F_E_(2, 1)) + "\t" + std::to_string(F_E_(2, 2)) + "\n\n";

	// determinant
	ret += "J_E:\t" + std::to_string(F_E_.determinant()) + "\n";

	// hardening
	ret += "q:\t" + std::to_string(q_) + "\n";
	ret += "alpha\t:" + std::to_string(alpha_) + "\n";

	return ret;
}