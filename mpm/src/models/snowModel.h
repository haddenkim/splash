#pragma once

#include "models/constitutiveModel.h"

class SnowModel : public ConstitutiveModel {
public:
	SnowModel(double vol0);

	void updateDeformation(const Eigen::Matrix3d& velGradient, const double timestep) override;

	double			computePotentialEnergy() const override;
	Eigen::Matrix3d computeVolCauchyStress() const override;
	void			computeMuLambda(double& mu, double& lambda, const double& J_P) const;

	// for rendering
	virtual double getElastic() const override;
	virtual double getPlastic() const override;
	
	// for GUI
	std::string getGui() const ;

	// data
	double			vol0_;
	Eigen::Matrix3d F_E_;
	Eigen::Matrix3d F_P_;
	Eigen::Matrix3d R_E_;

	// material properties
	static const double thetaC_;  // θ_c critical compression
	static const double thetaS_;  // θ_s critical stretch
	static const double xi_;	  // ξ hardening coefficient
	static const double E0_;	  // E initial Young's Modulus
	static const double nu_;	  // ν Poisson ratio
	static const double mu0_;	 // μ shear modulus
	static const double lambda0_; // λ Lame's first parameter
};