#pragma once
#include "models/constitutiveModel.h"

class SnowModel : public ConstitutiveModel {
public:
	// Stomakhin Snow model and default parameters (stom Table 2)
	// lowered E from 1.4e5 to 1e4 to induce more fracturing
	SnowModel(double thetaC = 2.5e-2, // θ_c critical compression
			  double thetaS = 7.5e-3, // θ_s critical stretch
			  double xi		= 10.0,   // ξ hardening coefficient
			  double E0		= 1e4,	// E_0 initial Young's Modulus
			  double nu		= 0.2);		  // ν Poisson ratio

	// computes the elastic potential energy
	double computePotentialEnergy(const Eigen::Matrix3d& F_E,
								  const Eigen::Matrix3d& R_E,
								  const Eigen::Matrix3d& F_P,
								  double				 vol0) const override;

	// computes and updates the needed deformation gradients
	void updateDeformDecomp(Eigen::Matrix3d&	   F_E,
							Eigen::Matrix3d&	   R_E,
							Eigen::Matrix3d&	   F_P,
							double&				   J_P,
							const Eigen::Matrix3d& velGradient,
							const double&		   timestep) const override;

	// computes current volume * cauchy stress V_p * σ_p
	Eigen::Matrix3d computeVolCauchyStress(const double&		  vol0,
										   const Eigen::Matrix3d& F_E,
										   const Eigen::Matrix3d& R_E,
										   const double&		  J_P) const override;

	// computes First Piola Kirchoff Differential δP
	Eigen::Matrix3d computeFirstPiolaKirchoffDifferential(const Eigen::Matrix3d& differentialF_E,
														  const Eigen::Matrix3d& F_E,
														  const double&			 J_P) const override;

private:
	// helper functions
	void computeMuLambda(double& mu, double& lambda, const double& J_P) const;

	Eigen::Matrix3d computeDifferentialJFinvT(const Eigen::Matrix3d& differentialF_E, const Eigen::Matrix3d& F_E) const;

	Eigen::Matrix3d computeDifferentialR(const Eigen::Matrix3d& differentialF_E,
										 const Eigen::Matrix3d& R_E,
										 const Eigen::Matrix3d& S_E) const;

	double thetaC_;  // θ_c critical compression
	double thetaS_;  // θ_s critical stretch
	double xi_;		 // ξ hardening coefficient
	double mu0_;	 // μ shear modulus
	double lambda0_; // λ Lame's first parameter
};