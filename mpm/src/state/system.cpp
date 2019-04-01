#include "system.h"

#include <cassert>
#include <vector>

using namespace Eigen;

System::System(Eigen::Vector3d cellSize, Eigen::Vector3d worldSize)
	: cellSize_(cellSize)
	, worldSize_(worldSize)
{
	initGrid();
	computeKernelSize();
}

void System::addParticle(double m, double vol, Eigen::Vector3d position, Eigen::Vector3d velocity)
{
	particles_.emplace_back(Particle(m,
									 vol,
									 position,
									 velocity));
}

void System::clearParticles()
{
	particles_.clear();
}

Node* System::getNodeAt(int x, int y, int z)
{
	int index = z * (gridSize_.x() * gridSize_.y()) + y * gridSize_.x() + x;

	assert(index < nodes_.size());

	return &nodes_[index];
}

void System::initGrid()
{
	// check grid size
	assert(fmod(worldSize_.x(), cellSize_.x()) == 0.0 && fmod(worldSize_.y(), cellSize_.y()) == 0.0 && fmod(worldSize_.z(), cellSize_.z()) == 0.0);

	// integer grid size
	gridSize_ = worldSize_.cwiseQuotient(cellSize_).cast<int>() + Vector3i(1, 1, 1);

	// initialize nodes
	int ni = 0;

	for (double k = 0.0; k <= worldSize_.z(); k += cellSize_.z()) {
		for (double j = 0.0; j <= worldSize_.y(); j += cellSize_.y()) {
			for (double i = 0.0; i <= worldSize_.x(); i += cellSize_.x()) {

				nodes_.emplace_back(Node(ni, Vector3d(i, j, k)));
				ni++;
			}
		}
	}
}

void System::computeKernelSize()
{
	// TODO: investigate user-variable kernels
	// for now, fix kernel world size to (2,2,2) (radius = 1)
	kernelRadius_		  = 1.0;
	double kernelDiameter = kernelRadius_ * 2.0;

	// check size
	assert(fmod(kernelDiameter, cellSize_.x()) == 0.0 && fmod(kernelDiameter, cellSize_.y()) == 0.0 && fmod(kernelDiameter, cellSize_.z()) == 0.0);

	// integer kernel size
	kernelSize_ = Vector3d(kernelDiameter, kernelDiameter, kernelDiameter).cwiseQuotient(cellSize_).cast<int>() + Vector3i(1, 1, 1);

	// kernel offset
	kernelOffset_ = kernelSize_ / 2;
}