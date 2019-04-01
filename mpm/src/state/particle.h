#pragma once

#include <Eigen/Core>

struct Particle {

	Particle(double mass, double volume, Eigen::Vector3d position, Eigen::Vector3d velocity)
		: mass(mass)
		, volume(volume)
		, position(position)
		, velocity(velocity)
	{
		// TODO: engineer constructors or subclasses to allow alternative material properties and/or types

		// For now, particle is snow
		// from Stomakhin 2013 MPM Snow Table 2
		criticalCompression  = 2.5e-2;
		criticalStretch		 = 7.5e-3;
		hardening			 = 10.0;
		double youngsModulus = 1.4e+5; // E
		double poissonsRatio = 0.2;	// ν

		// compute and store initial mu and lambda (eq 47)
		mu0		= youngsModulus / (2 * (1 + poissonsRatio));
		lambda0 = youngsModulus * poissonsRatio / ((1 + poissonsRatio) * (1 - 2 * poissonsRatio));

		// set inital affine state
		affineState.setZero();

		// set inital deformation state
		// deformationGradient.setIdentity();
		elasticGradient.setIdentity();
		elasticDeterminant = 1.0;
		plasticGradient.setIdentity();
		plasticDeterminant = 1.0;
		elasticRotation.setIdentity();
		// symmetric.setIdentity();

		// plasticDeterminant = 1.0;
	}

	// TODO: consider converting this to static variables or sim parameters, as they do not change per particle (unless simulating heterogenous material)
	double criticalCompression; // θ_c
	double criticalStretch;		// θ_s
	double hardening;			// ξ
	double mu0;					// μ shear modulus
	double lambda0;				// λ Lame's first parameter

	// fixed state
	double mass;   // m
	double volume; // V

	// time dependent state
	Eigen::Vector3d position; // x
	Eigen::Vector3d velocity; // v

	Eigen::Matrix3d affineState; // B

	// Eigen::Matrix3d deformationGradient; // F
	Eigen::Matrix3d elasticGradient;	//# F_E elastic part of deformation gradient F
	double			elasticDeterminant; //# J_E jacobian determinant of elastic gradient
	Eigen::Matrix3d plasticGradient;	// F_P plastic part of deformation gradient F
	double			plasticDeterminant; //# J_P jacobian determinant of plastic gradient
	Eigen::Matrix3d elasticRotation;	//# R_E rotation component of polar decomposition of elastic gradient F
	// Eigen::Matrix3d symmetric;			 // S symmetric component of polar decomposition of deformation gradient F

	// time integration bookkeeping
	Eigen::Matrix3d nodalForceContribution;
	// Eigen::Matrix3d pk1Stress; // P - first Piola-Kirchoff Stress
};
