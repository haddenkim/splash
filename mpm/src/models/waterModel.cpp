#include "models/waterModel.h"
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

// Becker and Teschner Weakly compressible SPH for free surface flows 3.1
const double WaterModel::gamma_ = 7;
const double WaterModel::k_		= 1119;

WaterModel::WaterModel(double vol0)
	: J_(1)
{
}

void WaterModel::updateDeformation(const Eigen::Matrix3d& velGradient, const double timestep)
{
	// Tamp eq 12

}

double WaterModel::computePotentialEnergy() const
{
}

Eigen::Matrix3d WaterModel::computeVolCauchyStress() const
{
}

void WaterModel::computeMuLambda(double& mu, double& lambda, const double& J_P) const
{
}

// for rendering
double WaterModel::getElastic() const
{
}

double WaterModel::getPlastic() const
{
}