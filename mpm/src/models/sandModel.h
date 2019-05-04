#pragma once

#include "models/constitutiveModel.h"

// Drucker-Prager Elastoplasticity for Sand Animation

class SandModel : public ConstitutiveModel {
public:
	SandModel(double vol0);

	void updateDeformation(const Eigen::Matrix3d& velGradient, const double timestep) override;

	double			computePotentialEnergy() const override;
	Eigen::Matrix3d computeVolCauchyStress() const override;

	// given Σ and α , computes T (stored in Sig) and δq (stored in alpha)
	void   projectToYieldSurface(Eigen::Vector3d& Sig, double& alpha) const;
	double computeHardening(double q) const;

	// for rendering
	virtual double getRenderElastic() const override;
	virtual double getRenderPlastic() const override;

	// for GUI
	std::string getGui() const ;

	// data
	double			vol0_;  // initial volume
	Eigen::Matrix3d F_E_;   // elastic deformation gradient
	double			alpha_; // yield surface size
	double			q_;		// hardening state

	// material properties
	static const double E_;		 // E Young's Modulus
	static const double nu_;	 // ν Poisson ratio
	static const double mu_;	 // μ shear modulus
	static const double lambda_; // λ Lame's first parameter

	static const double h0_;
	static const double h1_;
	static const double h2_;
	static const double h3_;

	// precompute for convenience/optimiation
	static const double sqrtTwoThirds_;
};