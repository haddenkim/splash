#pragma once
#include "models/modelType.h"
#include <Eigen/Dense>
#include <vector>

class System;
class SystemStart;
class Shape;

class SystemSetup {
public:
	static void initSystem(System& system, const SystemStart& start, int partCount);

	//helpers
	static void clearParticles(System& system);
	static void setupGrid(System& system);
	static void addShapes(System& system, const std::vector<Shape>& shapes, int partCount);
	static void addPart(System& system, ModelType type, Eigen::Vector3d pos, Eigen::Vector3d velocity, Eigen::RowVector3d color);
};