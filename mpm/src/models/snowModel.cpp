#include "snowModel.h"
#include "tensor/tensorHelpers.h"
#include <cmath>

using namespace Eigen;

SnowModel::SnowModel(double thetaC,
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

void SnowModel::updateDeformDecomp(Eigen::Matrix3d&		  F_E,
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

Eigen::Matrix3d SnowModel::computeVolCauchyStress(const double&			 vol0,
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

// computes First Piola Kirchoff Differential δP
Eigen::Matrix3d SnowModel::computeFirstPiolaKirchoffDifferential(const Eigen::Matrix3d& differentialF_E,
																 const Eigen::Matrix3d& F_E,
																 const double&			J_P) const
{
	// δP = [∂P/∂F]:δF

	// compute current mu and lambda (eq 87)
	double mu;
	double lambda;
	computeMuLambda(mu, lambda, J_P);

	// SVD of F_E (eq 83)
	JacobiSVD<Matrix3d> svd(F_E, ComputeFullU | ComputeFullV);
	Matrix3d			U   = svd.matrixU();		// U
	Vector3d			Sig = svd.singularValues(); // Σ
	Matrix3d			V   = svd.matrixV();		// V

	// construct R (rotation) and S (symmetric) matrix (para above 52)
	Eigen::Matrix3d S_E = V * Sig.asDiagonal() * V.transpose();
	Eigen::Matrix3d R_E = U * V.transpose();

	// precompute for convenience
	double   J_E	  = F_E.determinant();
	Matrix3d JFinvT_E = J_E * F_E.inverse().transpose(); // JF^-T

	// compute differential R
	Matrix3d differentialR_E = computeDifferentialR(differentialF_E, R_E, S_E);

	// compute differential JF^-T
	Matrix3d differentialJFinvT = computeDifferentialJFinvT(differentialF_E, F_E);

	// compute δP (eq 56) aka A_p (eq 197)
	return 2 * mu * (differentialF_E - differentialR_E)
		+ lambda * JFinvT_E * doubleContraction(JFinvT_E, differentialF_E)
		+ lambda * (J_E - 1) * differentialJFinvT;
}

void SnowModel::computeMuLambda(double& mu, double& lambda, const double& J_P) const
{
	// compute current mu and lambda (eq 87)
	double exp = std::exp(xi_ * (1.0 - J_P));
	mu		   = mu0_ * exp;
	lambda	 = lambda0_ * exp;
}

Eigen::Matrix3d SnowModel::computeDifferentialJFinvT(const Eigen::Matrix3d& differentialF_E, const Eigen::Matrix3d& F_E) const
{
	// compute δ(JF^-T) (para after eq 56)
	// δ(JF^-T) = ∂(JF^-T)/∂F : δF  (para after eq 56)

	// matlab output
	// dFunc_dF_ddot_dF =
	// [ F2_2*dF3_3 - F2_3*dF3_2 - F3_2*dF2_3 + F3_3*dF2_2,   F2_3*dF3_1 - F2_1*dF3_3 + F3_1*dF2_3 - F3_3*dF2_1,   F2_1*dF3_2 - F2_2*dF3_1 - F3_1*dF2_2 + F3_2*dF2_1]
	// [ F1_3*dF3_2 - F1_2*dF3_3 + F3_2*dF1_3 - F3_3*dF1_2,   F1_1*dF3_3 - F1_3*dF3_1 - F3_1*dF1_3 + F3_3*dF1_1,   F1_2*dF3_1 - F1_1*dF3_2 + F3_1*dF1_2 - F3_2*dF1_1]
	// [ F1_2*dF2_3 - F1_3*dF2_2 - F2_2*dF1_3 + F2_3*dF1_2,   F1_3*dF2_1 - F1_1*dF2_3 + F2_1*dF1_3 - F2_3*dF1_1,   F1_1*dF2_2 - F1_2*dF2_1 - F2_1*dF1_2 + F2_2*dF1_1]

	// for convenience (should be opimized away)
	const Matrix3d& F  = F_E;
	const Matrix3d& dF = differentialF_E;

	Matrix3d ret;
	ret(0, 0) = F(1, 1) * dF(2, 2) - F(1, 2) * dF(2, 1) - F(2, 1) * dF(1, 2) + F(2, 2) * dF(1, 1);
	ret(0, 1) = F(1, 2) * dF(2, 0) - F(1, 0) * dF(2, 2) + F(2, 0) * dF(1, 2) - F(2, 2) * dF(1, 0);
	ret(0, 2) = F(1, 0) * dF(2, 1) - F(1, 1) * dF(2, 0) - F(2, 0) * dF(1, 1) + F(2, 1) * dF(1, 0);

	ret(1, 0) = F(0, 2) * dF(2, 1) - F(0, 1) * dF(2, 2) + F(2, 1) * dF(0, 2) - F(2, 2) * dF(0, 1);
	ret(1, 1) = F(0, 0) * dF(2, 2) - F(0, 2) * dF(2, 0) - F(2, 0) * dF(0, 2) + F(2, 2) * dF(0, 0);
	ret(1, 2) = F(0, 1) * dF(2, 0) - F(0, 0) * dF(2, 1) + F(2, 0) * dF(0, 1) - F(2, 1) * dF(0, 0);

	ret(2, 0) = F(0, 1) * dF(1, 2) - F(0, 2) * dF(1, 1) - F(1, 1) * dF(0, 2) + F(1, 2) * dF(0, 1);
	ret(2, 1) = F(0, 2) * dF(1, 0) - F(0, 0) * dF(1, 2) + F(1, 0) * dF(0, 2) - F(1, 2) * dF(0, 0);
	ret(2, 2) = F(0, 0) * dF(1, 1) - F(0, 1) * dF(1, 0) - F(1, 0) * dF(0, 1) + F(1, 1) * dF(0, 0);

	return ret;
}

Eigen::Matrix3d SnowModel::computeDifferentialR(const Eigen::Matrix3d& differentialF_E,
												const Eigen::Matrix3d& R_E,
												const Eigen::Matrix3d& S_E) const
{

	// compute R^T δF - δF^T R (eq 59 lhs)
	Matrix3d lhs = R_E.transpose() * differentialF_E - differentialF_E.transpose() * R_E;

	// compute R^T δR components
	// lhs = [R^T δR]S + S[R^T δR] (eq 59)
	// |  0  d  e  |   |  0  a  b  | | 		|	 |		| |  0  a  b  |
	// | -d  0  f  | = | -a  0  c  | |	S	|  + |	S	| | -a  0  c  |
	// | -e -f  0  |   | -b -c  0  | |		|	 |		| | -b -c  0  |

	// from matlab
	// d = a*S1_1 + a*S2_2 - c*S1_3 + b*S3_2
	// e = b*S1_1 + a*S2_3 + c*S1_2 + b*S3_3
	// f = b*S2_1 - a*S1_3 + c*S2_2 + c*S3_3

	// [ S1_1 + S2_2,	S3_2,			-S1_3		]
	// [ S2_3,			S1_1 + S3_3,	S1_2		]
	// [ -S1_3,        	S2_1, 			S2_2 + S3_3	]

	Matrix3d eqs;
	eqs << S_E(0, 0) + S_E(1, 1), S_E(2, 1), -S_E(0, 2),
		S_E(1, 2), S_E(0, 0) + S_E(2, 2), S_E(0, 1),
		-S_E(0, 2), S_E(1, 0), S_E(1, 1) + S_E(2, 2);

	Vector3d vars = Vector3d(lhs(0, 1), lhs(0, 2), lhs(1, 2));

	// solve system of equations
	vars = eqs.inverse() * vars;

	// construct R^T δR
	// |  0  a  b  |
	// | -a  0  c  |
	// | -b -c  0  |
	Matrix3d RTdR;
	RTdR << 0, vars(0), vars(1), -vars(0), 0, vars(2), -vars(1), -vars(2), 0;

	// compute δR (para after eq 59)
	return R_E * RTdR;
}