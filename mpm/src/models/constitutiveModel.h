#pragma once

#include <Eigen/Core>

class ConstitutiveModel {
public:
	virtual void updateDeformation(const Eigen::Matrix3d& velGradient, const double timestep) = 0;

	virtual double			computePotentialEnergy() const = 0;
	virtual Eigen::Matrix3d computeVolCauchyStress() const = 0;

	// for rendering
	virtual double getRenderElastic() const = 0;
	virtual double getRenderPlastic() const = 0;

	// for GUI
	virtual std::string getGui() const = 0;
};
