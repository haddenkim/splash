#pragma once
#include "kernels/interpolation.h"
#include "models/constitutiveModel.h"
#include <Eigen/Core>

struct Particle {
	Particle(Eigen::Vector3d x, Eigen::Vector3d v, Eigen::RowVector3d color, ConstitutiveModel* model)
		: pos(x)
		, vel(v)
		, color(color)
		, model(model)
	{
		// TODO engineer way to vary these parameters or compute them (ex. FEM style eq 136-155)
		mass0 = 1.0;
		vol0  = 1.0;

		B.setZero();
		F_E.setIdentity();
		F_P.setIdentity();
		R_E.setIdentity();
		J_P = 1.0;
	}

	// rendering
	Eigen::RowVector3d color; // rendering color

	// transfer related
	double			mass0; // intial mass
	Eigen::Vector3d pos;   // position
	Eigen::Vector3d vel;   // velocity
	Eigen::Matrix3d B;	 // affine state

	// weight -> transfer, wGrad -> force
	Interpolation kernel;

	// force related
	ConstitutiveModel* model; //

	double			vol0; // initial volume
	Eigen::Matrix3d F_E;  // elastic part of deformation gradient F
	Eigen::Matrix3d F_P;  // plastic part of deformation gradient F
	Eigen::Matrix3d R_E;  // rotational decomposition of F_E
	double			J_P;  // determinant of the plastic deformation gradient

	Eigen::Matrix3d VPFT;		 // V_p * P(F_p) * (F_p)^T (eq 189)
	Eigen::Matrix3d velGradient; // accumulation of ∇v_p = Σ_i(v_i * ∇w_ip) (eq 181) (stomakhin step 7)

	// only used by implicit solvers
	Eigen::Matrix3d VAFT; // V_p * A_p * (F_p)^T (eq 196)
};