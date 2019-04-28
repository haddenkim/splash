#include "system.h"
#include "models/sandModel.h"
#include "models/snowModel.h"
#include <Eigen/Dense>
#include <algorithm>

using namespace Eigen;

void System::restart(SystemStart start)
{
	// clear old data
	constitutiveModels.clear();
	particles_.clear();

	// setup grid
	nodes_ = std::vector<Node>(WORLD_NUM_NODES);
	setupGrid();

	// setup models
	constitutiveModels.emplace_back(new SnowModel()); // snow 0
	constitutiveModels.emplace_back(new SandModel()); // sand 1

	// setup particles
	addShapes(start.shapes);

	// setup boundary conditions
	boundaryStart_ = 3;
	boundaryEnd_   = WORLD_NUM_NODES_Z - 3;

	// sort particles
	sortParticles();
}

int System::partCount() const
{
	return particles_.size();
}

Particle& System::getPart(int pi)
{
	return particles_[pi];
}

const Particle& System::getPart(int pi) const
{
	return particles_[pi];
}

int System::nodeCount() const
{
	return nodes_.size();
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

Node& System::getNode(int ni)
{
	return nodes_[ni];
}

const Node& System::getNode(int ni) const
{
	return nodes_[ni];
}

Node& System::getNode(int x, int y, int z)
{
	int ni = getNodeIndex(x, y, z);

	assert(ni >= 0 && ni < nodes_.size());

	return getNode(ni);
}

Node& System::getNode(Eigen::Vector3i pos)
{
	return getNode(pos.x(), pos.y(), pos.z());
}

Node& System::getNode(Eigen::Vector3d pos)
{
	// position of nearest node
	Eigen::Vector3i nodePos = (pos.array() + 0.5).cast<int>();

	return getNode(nodePos);
}

bool System::isInBounds(double x, double y, double z) const
{
	return x >= 0 && y >= 0 && z >= 0 && x < WORLD_NUM_NODES_X && y < WORLD_NUM_NODES_Y && z < WORLD_NUM_NODES_Z;
}

bool System::isInBounds(Eigen::Vector3d pos) const
{
	return isInBounds(pos.x(), pos.y(), pos.z());
}

void System::sortParticles()
{
	// sort particles based on their node's index
	std::sort(particles_.begin(), particles_.end(), [&](const Particle& a, const Particle& b) -> bool {
		Eigen::Vector3i aNode = (a.pos.array() + 0.5).cast<int>();
		Eigen::Vector3i bNode = (b.pos.array() + 0.5).cast<int>();

		return getNodeIndex(aNode.x(), aNode.y(), aNode.z()) > getNodeIndex(bNode.x(), bNode.y(), bNode.z());
	});

	// re-assign node ownership
	for (Node& node : nodes_) {
		node.ownedParticles.clear();
	}

	for (int pi = 0; pi < particles_.size(); pi++) {
		const Particle& part = particles_[pi];

		// position of nearest node
		Eigen::Vector3i nodePos = (part.pos.array() + 0.5).cast<int>();

		// add to node's list
		Node& node = getNode(nodePos);
		node.ownedParticles.insert(&particles_[pi]);
	}
}

void System::setupGrid()
{

	// set every node's grid position
	int ni;
	for (int x = 0; x < WORLD_NUM_NODES_X; x++) {
		for (int y = 0; y < WORLD_NUM_NODES_Y; y++) {
			for (int z = 0; z < WORLD_NUM_NODES_Z; z++) {

				int   ni   = getNodeIndex(x, y, z);
				Node& node = nodes_[ni];

				// position
				node.x = x;
				node.y = y;
				node.z = z;

				// neighbors
				for (int ki = 0; ki < KERNEL_NUM_NODES_X; ki++) {
					for (int kj = 0; kj < KERNEL_NUM_NODES_X; kj++) {
						for (int kk = 0; kk < KERNEL_NUM_NODES_X; kk++) {
							// compute neighbor position
							int kx = x + ki - 1;
							int ky = y + kj - 1;
							int kz = z + kk - 1;

							// index in node's neighbor list
							int index = ki * 9 + kj * 3 + kk;

							// add to neighbor list (or nullptr if out of bounds)
							node.neighbors[index] = isInBounds(kx, ky, kz) ? &getNode(kx, ky, kz) : nullptr;
						}
					}
				}
			}
		}
	}
}

void System::addShapes(std::vector<Shape> shapes)
{
	for (Shape& shape : shapes) {
		for (int pi = 0; pi < PARTS_PER_OBJECT; pi++) {
			Vector3d pos = shape.getRandomParticlePos();

			addPart(shape.type, pos, shape.velocity, shape.color);
		}
	}
}

void System::addPart(ModelType type, Eigen::Vector3d pos, Eigen::Vector3d velocity, Eigen::RowVector3d color)
{

	// retrieve model
	ConstitutiveModel* model = constitutiveModels[0];

	particles_.emplace_back(Particle(pos, velocity, color, model));
}