#include "state/systemSetup.h"
#include "settings/constants.h"
#include "settings/systemStart.h"
#include "state/shape.h"
#include "state/system.h"
#include "state/systemHelpers.h"

#include "models/sandModel.h"
#include "models/snowModel.h"

using namespace Eigen;

void SystemSetup::initSystem(System& system, const SystemStart& start, int partCount)
{
	// clear existing data
	clearParticles(system);

	// setup grid
	setupGrid(system);

	// setup models
	system.constitutiveModels.emplace_back(new SnowModel()); // snow 0
	system.constitutiveModels.emplace_back(new SandModel()); // sand 1

	// setup particles
	addShapes(system, start.shapes, partCount);

	// setup boundary conditions
	system.boundaryStart = 3;
	system.boundaryEnd   = WORLD_NUM_NODES_Z - 3;

	// sort particles
}

void SystemSetup::clearParticles(System& system)
{
	system.partCount = 0;

	system.particles.clear();
	system.partPos.clear();

	system.partMass.clear();
	system.partVel.clear();
	system.partB.clear();

	system.partModel.clear();
	system.partVol0.clear();
	system.partF_E.clear();
	system.partF_P.clear();
	system.partR_E.clear();
	system.partJ_P.clear();

	system.partColor.clear();

	system.partVelGrad.clear();
	system.partReorderBuffer.clear();

	system.constitutiveModels.clear();
}

void SystemSetup::setupGrid(System& system)
{
	// set node positions and neighbors
	for (int x = 0; x < WORLD_NUM_NODES_X; x++) {
		for (int y = 0; y < WORLD_NUM_NODES_Y; y++) {
			for (int z = 0; z < WORLD_NUM_NODES_Z; z++) {

				int   ni   = computeNodeIndex(x, y, z);
				Node& node = system.nodes[ni];

				// position
				node.pos = Vector3i(x, y, z);

				// neighbors
				system.neighbors[ni] = computeNodeNeighbors(x, y, z, system.nodes);
			}
		}
	}

	// set node blocks and node sets
	int bi  = 0; // block index
	int si  = 0; // set index
	int sbi = 0; // index of block in set

	for (int ni = 0; ni < WORLD_NUM_NODES; ni += BLOCK_NUM_NODES) {

		system.sets[si][sbi] = computeBlockIndex(ni);

		// increment or wrap set index
		if (si != 7) {
			si++;
		} else {
			si = 0;
			sbi++;
		}
	}
}

void SystemSetup::addShapes(System& system, const std::vector<Shape>& shapes, int partCount)
{
	for (const Shape& shape : shapes) {
		for (int pi = 0; pi < partCount; pi++) {
			Vector3d pos = shape.getRandomParticlePos();

			addPart(system, shape.type, pos, shape.velocity, shape.color);
		}
	}
}

void SystemSetup::addPart(System& system, ModelType type, Eigen::Vector3d pos, Eigen::Vector3d velocity, Eigen::RowVector3d color)
{
	// TODO allow for alternative initial state
	double mass  = 1;
	double vol   = 1;
	auto   model = system.constitutiveModels[0];

	int bi = computeBlockIndex(pos);

	system.particles.push_back(ParticleBase(system.partCount, bi));
	system.partCount++;

	system.partPos.push_back(pos);

	system.partMass.push_back(mass);
	system.partVel.push_back(velocity);
	system.partB.push_back(Matrix3d::Zero());

	system.partModel.push_back(model);
	system.partVol0.push_back(vol);
	system.partF_E.push_back(Matrix3d::Identity());
	system.partF_P.push_back(Matrix3d::Identity());
	system.partR_E.push_back(Matrix3d::Identity());
	system.partJ_P.push_back(1);

	system.partColor.push_back(color);

	system.partVelGrad.push_back(Matrix3d::Zero());

	system.partReorderBuffer.insert(system.partReorderBuffer.end(),
									{ 0, 0, 0, 0, 0, 0, 0, 0, 0 });
}