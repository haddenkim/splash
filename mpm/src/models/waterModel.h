#pragma once

#include "models/constitutiveModel.h"

class WaterModel : public ConstitutiveModel {
public:
	WaterModel(double vol0);

	void updateDeformation(const Eigen::Matrix3d& velGradient, const double timestep) override;

	double			computePotentialEnergy() const override;
	Eigen::Matrix3d computeVolCauchyStress() const override;
	void			computeMuLambda(double& mu, double& lambda, const double& J_P) const;

	// for rendering
	virtual double getRenderElastic() const override;
	virtual double getRenderPlastic() const override;

	// data
	double vol0_; //
	double J_;	// deformation gradient determinant

	// material properties
	static const double gamma_; // "term that more stiffly penalizes large deviations from incompressibility"
	static const double k_;		// bulk modulus
};