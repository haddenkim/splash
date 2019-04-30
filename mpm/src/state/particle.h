#pragma once
#include "models/constitutiveModel.h"
#include <Eigen/Core>

struct ParticleBase {
	ParticleBase(int dataIndex, int blockIndex)
		: dataIndex(dataIndex)
		, blockIndex(blockIndex){};

	int dataIndex;
	int blockIndex;
};

// struct ParticleMomentum {
// 	ParticleMomentum(double mass0, Eigen::Vector3d vel)
// 		: mass0(mass0)
// 		, vel(vel)
// 		, B(Eigen::Matrix3d::Zero()){};

// 	double			mass0; // initial mass
// 	Eigen::Vector3d vel;   // velocity
// 	Eigen::Matrix3d B;	 // affine state
// };

// struct ParticleDeform {
// 	ParticleDeform(ConstitutiveModel* model)
// 		: model(model)
// 		, vol0(1)
// 		, F_E(Eigen::Matrix3d::Identity())
// 		, R_E(Eigen::Matrix3d::Identity())
// 		, F_P(Eigen::Matrix3d::Identity())
// 		, J_P(1){};

// 	ConstitutiveModel* model; //

// 	double			vol0; // initial volume
// 	Eigen::Matrix3d F_E;  // elastic part of deformation gradient F
// 	Eigen::Matrix3d F_P;  // plastic part of deformation gradient F
// 	Eigen::Matrix3d R_E;  // rotational decomposition of F_E
// 	double			J_P;  // determinant of the plastic deformation gradient
// };

// struct ParticleInterpolation {
// 	double			w[3][3][3];		  // w_aip
// 	Eigen::Vector3d wGrad_H[3][3][3]; // ∇w_aip
// };
