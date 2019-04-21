#pragma once
#include "models/constitutiveModel.h"

class SandModel : public ConstitutiveModel {
public:
	// Drucker-Prager sand
	SandModel();

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

};