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
										   const double&		  timestep) const
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
															 const double&			J_P) const
{
	// compute current mu and lambda (eq 87)
	double mu;
	double lambda;
	computeMuLambda(mu, lambda, J_P);

	// compute determinant of elastic deformation gradient J_E
	double J_E = F_E.determinant();

	// compute first Piola-Kirchoff Stress P (eq 52)
	Matrix3d P_hat = 2 * mu * (F_E - R_E) + (lambda * (J_E - 1) * J_E) * F_E.transpose();
}

Eigen::Matrix3d ConstitutiveModel::computeVolCauchyStress(const double&			 vol0,
														  const Eigen::Matrix3d& F_E,
														  const Eigen::Matrix3d& R_E,
														  const double&			 J_P) const
{
	// compute current mu and lambda (eq 87)
	double mu;
	double lambda;
	computeMuLambda(mu, lambda, J_P);

	// compute determinant of elastic deformation gradient J_E
	double J_E = F_E.determinant();

	// compute the per part constant contribution to nodal elastic force V_p * P_p * (F_p)^T (eq 189)
	// aka current volume * cauchy stress V_p * σ_p (eq 190)
	Matrix3d PFT = 2 * mu * (F_E - R_E) * F_E.transpose() + (lambda * (J_E - 1) * J_E) * Matrix3d::Identity();

	return vol0 * PFT;
}

Eigen::MatrixXd ConstitutiveModel::computeFirstPiolaKirchoffDerivative(const Eigen::Matrix3d& F_E,
																	   const Eigen::Matrix3d& R_E,
																	   const double&		  J_P) const
{
	// compute current mu and lambda (eq 87)
	double mu;
	double lambda;
	computeMuLambda(mu, lambda, J_P);

	// compute U Σ V^T from svd F
	JacobiSVD<Matrix3d> svd(F_E, ComputeFullU | ComputeFullV);
	Matrix3d			U   = svd.matrixU();		// U
	Vector3d			Sig = svd.singularValues(); // Σ
	Matrix3d			V   = svd.matrixV();		// V

	double   J_S	= Sig.prod();			 // det(Σ)
	Vector3d SigInv = Sig.array().inverse(); // Σ^-1

	// compute P(Σ) aka ∂Ψ̂ /∂σ (sentence after eq 63)
	//   P(Σ) = μ(Σ - I) + λ(J_S -1)(J_S)Σ^-1
	Vector3d P_S = mu * (Sig.array() - 1).matrix() + lambda * (J_S - 1) * J_S * SigInv;

	// compute ∂2Ψ/∂2σ aka A (eq 75)
	//   ∂P̂/∂Σ = μ(∂Σ/∂Σ) + λ(J - 1)J (∂Σ^-1/∂Σ) + λ(2J - 1)(∂J/∂Σ)Σ^-1
	//   ∂P̂/∂Σ = (μ + λ(J - 1)J)I + λ(2J - 1)(∂J/∂Σ)Σ^-1
	Matrix3d dP_dSig = (mu * lambda * (J_S - 1) * J_S) * Matrix3d::Identity() + lambda * (2 * J_S - 1) * (J_S * SigInv * SigInv.transpose());

	//  construct ∂P/∂F (para above 75 )
	MatrixXd dP_dF(9, 9);
	dP_dF.setZero();
	dP_dF.block<3, 3>(0, 0) = dP_dSig;
	dP_dF.block<2, 2>(3, 3) = computeB_ij(1, 2, Sig, P_S);
	dP_dF.block<2, 2>(5, 5) = computeB_ij(1, 3, Sig, P_S);
	dP_dF.block<2, 2>(7, 7) = computeB_ij(2, 3, Sig, P_S);

	// TODO : investigate if better to reorder once here, vs calc index in hessian compute
	// reorder for convenience
	// index 		:0	, 1	 , 2  , 3  , 4  , 5  , 6  , 7  , 8
	// Stom order	:s11, s22, s33, s12, s21, s13, s31, s23, s32
	// Desired 		:s11, s12, s13, s21, s22, s23, s31, s32, s33
	// Operation	:n/a, 3  , 5  , 4  , 1  , 7  , n/a, 8  , 2   = 7 swaps

	// Desired 		:s11, s21, s31, s12, s22, s32, s13, s23, s33
	// Operation	:n/a, 4  , 6  , n/a, 1  , 8  , 5  , n/a, 2   = 6 swaps

	return dP_dF;
}

void ConstitutiveModel::computeMuLambda(double& mu, double& lambda, const double& J_P) const
{
	// compute current mu and lambda (eq 87)
	double exp = std::exp(xi_ * (1.0 - J_P));
	mu		   = mu0_ * exp;
	lambda	 = lambda0_ * exp;
}

Eigen::Matrix2d ConstitutiveModel::computeB_ij(int i, int j, const Eigen::Vector3d& Sig, const Eigen::Vector3d& P_S) const
{
	// clamp B denominators (para after 77)
	double denominatorL = std::max(Sig.coeff(i) - Sig.coeff(j), 1e-6);
	double denominatorR = std::max(Sig.coeff(i) + Sig.coeff(j), 1e-6);

	// compute B (eq 77)
	Matrix2d mR;
	mR << 1, -1, -1, 1;

	return 0.5 * (P_S.coeff(i) - P_S.coeff(j)) / denominatorL * Matrix2d::Ones()
		+ 0.5 * (P_S.coeff(i) + P_S.coeff(j)) / denominatorR * mR;
}
