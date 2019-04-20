#include "system.h"
#include "state/node.h"
#include <Eigen/Dense>
#include <algorithm>

using namespace Eigen;

System::System()
	: constitutiveModel_()
{
}

void System::reset(int size)
{
	particles_.clear();

	gridSize_  = size;
	nodeCountX = gridSize_ + 1;
	// dx_		  = 1.0 / (gridSize_ - 1);
	nodeCountXY_ = nodeCountX * nodeCountX;

	boundaryStart_ = 3;
	boundaryEnd_   = gridSize_ - 3;

	setupGrid(size);
}

void System::addCube(int partCount, Vector3d center, Vector3d velocity, RowVector3d color)
{
	double diameter = (double)gridSize_ / 10;

	for (int i = 0; i < partCount; i++) {
		// get new particle's index
		int pi = particles_.size();

		// random position from [-1,1]
		Vector3d position = Vector3d::Random().array();

		position = position * diameter + center * gridSize_;

		particles_.emplace_back(Particle(position, velocity, color));

		// position of nearest node
		Eigen::Vector3i nodePos = (position.array() + 0.5).cast<int>();

		// add to node's list
		Node* node = getNode(nodePos);
		node->ownedParticles.insert(pi);
	}
}

unsigned System::getNodeIndex(u_int32_t x, u_int32_t y, u_int32_t z) const
{
	// Morton order 16 bit
	// x = (x | (x << 16)) & 0x030000FF;
	x = (x | (x << 8)) & 0x0300F00F;
	x = (x | (x << 4)) & 0x030C30C3;
	x = (x | (x << 2)) & 0x09249249;

	// y = (y | (y << 16)) & 0x030000FF;
	y = (y | (y << 8)) & 0x0300F00F;
	y = (y | (y << 4)) & 0x030C30C3;
	y = (y | (y << 2)) & 0x09249249;

	// z = (z | (z << 16)) & 0x030000FF;
	z = (z | (z << 8)) & 0x0300F00F;
	z = (z | (z << 4)) & 0x030C30C3;
	z = (z | (z << 2)) & 0x09249249;

	return x | (y << 1) | (z << 2);

	// Z Y X
	// return x * nodeCountXY_ + y * nodeCountX + z;
}

Node* System::getNode(int x, int y, int z)
{
	int ni = getNodeIndex(x, y, z);

	assert(ni >= 0 && ni < nodes_.size());

	return &nodes_[getNodeIndex(x, y, z)];
}

Node* System::getNode(Eigen::Vector3i pos)
{
	return getNode(pos.x(), pos.y(), pos.z());
}

bool System::isInBounds(double x, double y, double z) const
{
	return x >= 0 || y >= 0 || z >= 0 || x < gridSize_ || y < gridSize_ || z < gridSize_;
}

bool System::isInBounds(Eigen::Vector3d pos) const
{
	return isInBounds(pos.x(), pos.y(), pos.z());
}

void System::setupGrid(int size)
{
	nodes_ = std::vector<Node>(nodeCountX * nodeCountX * nodeCountX);

	// set every node's grid position
	int ni;
	for (int i = 0; i < nodeCountX; i++) {
		for (int j = 0; j < nodeCountX; j++) {
			for (int k = 0; k < nodeCountX; k++) {

				int   ni   = getNodeIndex(i, j, k);
				Node& node = nodes_[ni];

				node.x = i;
				node.y = j;
				node.z = k;
			}
		}
	}
}

void System::sortParticles()
{
	std::sort(particles_.begin(), particles_.end(), [&](const Particle& a, const Particle& b) -> bool {
		Eigen::Vector3i aNode = (a.pos.array() + 0.5).cast<int>();
		Eigen::Vector3i bNode = (b.pos.array() + 0.5).cast<int>();

		return getNodeIndex(aNode.x(), aNode.y(), aNode.z()) > getNodeIndex(bNode.x(), bNode.y(), bNode.z());

	});
}
