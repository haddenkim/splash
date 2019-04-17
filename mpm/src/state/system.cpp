#include "system.h"
#include "state/node.h"
#include <Eigen/Dense>

using namespace Eigen;

System::System()
	: constitutiveModel_()
{
	setupGrid();
}

void System::clear()
{
	particles_.clear();
	for (int i = 0; i < gridSize_; i++) {
		for (int j = 0; j < gridSize_; j++) {
			for (int k = 0; k < gridSize_; k++) {

				// reference to node
				Node& node = nodes_[i][j][k];

				node.ownedParticles.clear();
			}
		}
	}
}

void System::addCube(int partCount, Vector3d center, Vector3d velocity, RowVector3d color)
{
	double scale = 0.1;

	for (int i = 0; i < partCount; i++) {

		Vector3d position = Vector3d::Random(3) * scale + center; // scale and translate

		particles_.emplace_back(Particle(position, velocity, color));

		// part position in grid frame
		Eigen::Vector3d partGridPos = position / dx_;

		// insert into nearest node
		Eigen::Vector3i nodeIndex = (partGridPos.array() + 0.5).cast<int>();
		Node&			node	  = nodes_[nodeIndex.x()][nodeIndex.y()][nodeIndex.z()];
		node.ownedParticles.insert(particles_.size() - 1);
	}
}

void System::setupGrid()
{
	// set every node's grid position
	for (int i = 0; i < gridSize_; i++) {
		for (int j = 0; j < gridSize_; j++) {
			for (int k = 0; k < gridSize_; k++) {

				Node& node = nodes_[i][j][k];
				node.x	 = i;
				node.y	 = j;
				node.z	 = k;
			}
		}
	}
}