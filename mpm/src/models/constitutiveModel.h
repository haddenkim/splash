#pragma once

#include <Eigen/Dense>

class ConstitutiveModel {
public:
	// TODO: consider subclassing this for alternative models
	// For now, Stomakhin Snow model and default parameters (stom Table 2)
	// lowered E0 from 1.4e5 because explicit solve was blowing up

	ConstitutiveModel(double thetaC = 2.5e-2, // θ_c critical compression
					  double thetaS = 7.5e-3, // θ_s critical stretch
					  double xi		= 10.0,   // ξ hardening coefficient
					  double E0		= 1e4,	// E_0 initial Young's Modulus
					  double nu		= 0.2);		  // ν Poisson ratiou

	// computes the needed deformation gradient decompositions without modifying input F_E and F_P (curr)
	void computeDeformDecomp(Eigen::Matrix3d&		F_E,
							 Eigen::Matrix3d&		R_E,
							 Eigen::Matrix3d&		F_P,
							 double&				J_P,
							 const Eigen::Matrix3d& F_E_curr,
							 const Eigen::Matrix3d& F_P_curr,
							 const Eigen::Matrix3d& velGradient,
							 const double&			timestep);

	// computes and updates the needed deformation gradients
	void updateDeformDecomp(Eigen::Matrix3d&	   F_E,
							Eigen::Matrix3d&	   R_E,
							Eigen::Matrix3d&	   F_P,
							double&				   J_P,
							const Eigen::Matrix3d& velGradient,
							const double&		   timestep);

	// computes First Piola-Kirchoff Stress P
	Eigen::Matrix3d computeFirstPiolaKirchoff(const Eigen::Matrix3d& F_E,
											  const Eigen::Matrix3d& R_E,
											  const double&			 J_P);

	// computes current volume * cauchy stress V_p * σ_p
	Eigen::Matrix3d computeVolCauchyStress(const double&		  vol0,
										   const Eigen::Matrix3d& F_E,
										   const Eigen::Matrix3d& R_E,
										   const double&		  J_P);

protected:
	double thetaC_;  // θ_c critical compression
	double thetaS_;  // θ_s critical stretch
	double xi_;		 // ξ hardening coefficient
	double mu0_;	 // μ shear modulus
	double lambda0_; // λ Lame's first parameter
};