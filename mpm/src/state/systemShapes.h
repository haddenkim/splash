#pragma once

#include "state/system.h"
#include <Eigen/Core>

class SystemShapes {
public:
	static void addCube(System& system, Eigen::Vector3d center, double radius, Eigen::Vector3d velocity, int count)
	{
		double partMass = 1.0;
		double partVol  = 1.0;

		for (int i = 0; i < count; i++) {
			double x = ((double)std::rand()) / (double)RAND_MAX; // between 0 - 1
			double y = ((double)std::rand()) / (double)RAND_MAX;
			double z = ((double)std::rand()) / (double)RAND_MAX;

			// position
			Eigen::Vector3d position(x, y, z);
			position *= radius * 2;										  // scale
			position += center - Eigen::Vector3d(radius, radius, radius); // translate

			system.addParticle(partMass, partVol, position, velocity);
		}
	};
};