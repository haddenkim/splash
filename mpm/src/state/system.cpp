#include "system.h"

#include <cassert>
#include <vector>

using namespace Eigen;

System::System(double cellSize, Eigen::Vector3d worldSize)
	: cellSize_(cellSize)
	, worldSize_(worldSize)
{
	initGrid();
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
	// integer grid size
	gridSize_ = (worldSize_ / cellSize_).cast<int>() + Vector3i(1, 1, 1);

	// initialize nodes
	int ni = 0;

	for (double k = 0.0; k <= worldSize_.z(); k += cellSize_) {
		for (double j = 0.0; j <= worldSize_.y(); j += cellSize_) {
			for (double i = 0.0; i <= worldSize_.x(); i += cellSize_) {

				nodes_.emplace_back(Node(ni, Vector3d(i, j, k)));
				ni++;
			}
		}
	}
}
