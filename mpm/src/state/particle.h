#pragma once

#include <Eigen/Core>

struct Particle {

	Particle(double mass, double volume, Eigen::Vector3d position, Eigen::Vector3d velocity)
		: mass(mass)
		, volume(volume)
		, position(position)
		, velocity(velocity)
	{
	}

	double mass;
	double volume;
	double mu;
	double lambda;


	// time dependent state
	Eigen::Vector3d position; // x
	Eigen::Vector3d velocity; // v

	Eigen::Matrix3d affineState;		 // B
	Eigen::Matrix3d deformationGradient; // F

	// time integration bookkeeping
	Eigen::Matrix3d nodalForceContribution;
	// Eigen::Matrix3d pk1Stress; // P - first Piola-Kirchoff Stress
};
