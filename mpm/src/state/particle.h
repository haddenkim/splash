#pragma once
#include <Eigen/Core>

struct Particle {
	Particle(Eigen::Vector3d x, Eigen::Vector3d v, Eigen::RowVector3d color)
		: pos(x)
		, vel(v)
		, color(color)
	{
		F.setIdentity();
		R.setIdentity();
		C.setZero();
		Jp = 1.0;

		B.setZero();
		F_E.setIdentity();
		F_P.setIdentity();
	}

	// Position and velocity
	Eigen::Vector3d pos; // position
	Eigen::Vector3d vel; // velocity

	Eigen::Matrix3d F;  // deformation gradient
	Eigen::Matrix3d R;  // rotational decomposition of F
	double			Jp; // determinant of the plastic deformation gradient

	// Affine momentum from APIC
	Eigen::Matrix3d C; //

	Eigen::RowVector3d color; // rendering color

	// TODO engineer way to vary these parameters
	const double mass = 1.0; // mass
	const double vol  = 1.0; // volume

	Eigen::Matrix3d B; // affine state

	Eigen::Matrix3d F_E; //  elastic part of deformation gradient F
	Eigen::Matrix3d F_P; //  plastic part of deformation gradient F

	// time integration bookkeeping
	Eigen::Vector3d w[3];			// w_aip
	Eigen::Vector3d wGrad[3]; // ∇w_aip

	Eigen::Matrix3d VPFT;			  // V_p * P(F_p) * (F_p)^T (eq 189)
	Eigen::Matrix3d velocityGradient; // accumulation of ∇v_p = Σ_i(v_i * ∇w_ip) (eq 181) (stomakhin step 7)
};