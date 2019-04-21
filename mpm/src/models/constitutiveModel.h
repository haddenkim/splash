#pragma once

#include <Eigen/Dense>

class ConstitutiveModel {
public:
	// computes the elastic potential energy
	virtual double computePotentialEnergy(const Eigen::Matrix3d& F_E,
										  const Eigen::Matrix3d& R_E,
										  const Eigen::Matrix3d& F_P,
										  double				 vol0) const = 0;

	// computes and updates the needed deformation gradients
	virtual void updateDeformDecomp(Eigen::Matrix3d&	   F_E,
									Eigen::Matrix3d&	   R_E,
									Eigen::Matrix3d&	   F_P,
									double&				   J_P,
									const Eigen::Matrix3d& velGradient,
									const double&		   timestep) const = 0;

	// computes current volume * cauchy stress V_p * σ_p
	virtual Eigen::Matrix3d computeVolCauchyStress(const double&		  vol0,
												   const Eigen::Matrix3d& F_E,
												   const Eigen::Matrix3d& R_E,
												   const double&		  J_P) const = 0;

	// computes First Piola Kirchoff Differential δP
	virtual Eigen::Matrix3d computeFirstPiolaKirchoffDifferential(const Eigen::Matrix3d& differentialF_E,
																  const Eigen::Matrix3d& F_E,
																  const double&			 J_P) const = 0;
};