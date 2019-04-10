#pragma once
#include "solver/interpolation.h"
#include <Eigen/Core>

struct Particle {
	Particle(Eigen::Vector3d x, Eigen::Vector3d v, Eigen::RowVector3d color)
		: pos(x)
		, vel(v)
		, color(color)
	{
		// TODO engineer way to vary these parameters or compute them (ex. FEM style eq 136-155)
		mass = 1.0;
		vol  = 1.0;

		B.setZero();
		F_E.setIdentity();
		F_P.setIdentity();
		R_E.setIdentity();
		J_P = 1.0;
	}

	// Time independent
	Eigen::RowVector3d color; // rendering color
	double			   mass;  // mass
	double			   vol;   // volume

	// time dependent
	Eigen::Vector3d pos; // position
	Eigen::Vector3d vel; // velocity

	Eigen::Matrix3d B; // affine state

	Eigen::Matrix3d F_E; // elastic part of deformation gradient F
	Eigen::Matrix3d F_P; // plastic part of deformation gradient F
	Eigen::Matrix3d R_E; // rotational decomposition of F_E
	double			J_P; // determinant of the plastic deformation gradient

	// time integration bookkeeping
	Interpolation kernel;

	Eigen::Matrix3d VPFT;		 // V_p * P(F_p) * (F_p)^T (eq 189)
	Eigen::Matrix3d velGradient; // accumulation of ∇v_p = Σ_i(v_i * ∇w_ip) (eq 181) (stomakhin step 7)

	Eigen::Matrix3d F_E_hat; // estimated F̂_E
};