#pragma once
#include "settings/constants.h"
#include <Eigen/Core>
#include <omp.h>

// computes the node index of a given node position
static u_int32_t computeNodeIndex(u_int32_t x, u_int32_t y, u_int32_t z)
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
}

// computes the node index of a given particle position
static u_int32_t computeNodeIndex(Eigen::Vector3d pos)
{
	// compute nearest node
	// and effectively rounds down to int
	int nx = pos.x() + 0.5;
	int ny = pos.y() + 0.5;
	int nz = pos.z() + 0.5;

	return computeNodeIndex(nx, ny, nz);
}

// computes the block index of a given node index
static u_int32_t computeBlockIndex(u_int32_t ni)
{
	return ni >> 6;
}

// computes the block index of a given node position
static u_int32_t computeBlockIndex(u_int32_t x, u_int32_t y, u_int32_t z)
{
	u_int32_t ni = computeNodeIndex(x, y, z);
	return computeBlockIndex(ni);
}

// computes the block index of a given particle position
static u_int32_t computeBlockIndex(Eigen::Vector3d pos)
{
	// compute nearest node
	u_int32_t ni = computeNodeIndex(pos);

	return computeBlockIndex(ni);
}

// // computes the root node of a given particle's kernel
// static u_int32_t computeKernelNode0(Eigen::Vector3d pos)
// {
// 	int kx = pos.x() - 0.5;
// 	int ky = pos.y() - 0.5;
// 	int kz = pos.y() - 0.5;

// 	return computeNodeIndex(kx, ky, kz);
// }

static bool isInBounds(int x, int y, int z)
{
	return x >= 0 && y >= 0 && z >= 0 && x < WORLD_NUM_NODES_X && y < WORLD_NUM_NODES_Y && z < WORLD_NUM_NODES_Z;
}

// sort particles (index only, not all particle data)
static void sortParticleIndices(System& system)
{
// update particle's node
#pragma omp parallel for schedule(static)
	for (int pi = 0; pi < system.partCount; pi++) {
		auto& part = system.particles[pi];
		auto& pos  = system.partPos[part.dataIndex];

		// update block index with current position
		part.blockIndex = computeBlockIndex(pos);
	}

	// TODO parallel sort
	// sort particles by updated index
	std::sort(system.particles.begin(), system.particles.end(), [&system](const ParticleBase& a, const ParticleBase& b) -> bool {
		return a.blockIndex < b.blockIndex;
	});

	system.partDataSorted = false;
}

// reorders a particle double vector based on the sorted particle indices
static void reorderParticleData(std::vector<double>& data, System& system)
{
#pragma omp parallel for schedule(static)
	for (int sortpi = 0; sortpi < system.partCount; sortpi++) {
		int pi = system.particles[sortpi].dataIndex;

		system.partReorderBuffer[sortpi] = data[pi];
	}

#pragma omp parallel for schedule(static)
	for (int pi = 0; pi < system.partCount; pi++) {
		data[pi] = system.partReorderBuffer[pi];
	}
}

// reorders a particle vec3d vector based on the sorted particle indices
static void reorderParticleData(std::vector<Eigen::Vector3d>& data, System& system)
{
	// #pragma omp parallel for schedule(static)
	for (int sortpi = 0; sortpi < system.partCount; sortpi++) {
		int pi = system.particles[sortpi].dataIndex;

		system.partReorderBuffer[sortpi * 3 + 0] = data[pi][0];
		system.partReorderBuffer[sortpi * 3 + 1] = data[pi][1];
		system.partReorderBuffer[sortpi * 3 + 2] = data[pi][2];
	}

	// #pragma omp parallel for schedule(static)
	for (int pi = 0; pi < system.partCount; pi++) {
		data[pi][0] = system.partReorderBuffer[pi * 3 + 0];
		data[pi][1] = system.partReorderBuffer[pi * 3 + 1];
		data[pi][2] = system.partReorderBuffer[pi * 3 + 2];
	}
}

// reorders a particle mat3d vector based on the sorted particle indices
static void reorderParticleData(std::vector<Eigen::Matrix3d>& data, System& system)
{
#pragma omp parallel for schedule(static)
	for (int sortpi = 0; sortpi < system.partCount; sortpi++) {
		int pi = system.particles[sortpi].dataIndex;

		system.partReorderBuffer[sortpi * 9 + 0] = data[pi](0, 0);
		system.partReorderBuffer[sortpi * 9 + 1] = data[pi](1, 0);
		system.partReorderBuffer[sortpi * 9 + 2] = data[pi](2, 0);

		system.partReorderBuffer[sortpi * 9 + 3] = data[pi](0, 1);
		system.partReorderBuffer[sortpi * 9 + 4] = data[pi](1, 1);
		system.partReorderBuffer[sortpi * 9 + 5] = data[pi](2, 1);

		system.partReorderBuffer[sortpi * 9 + 6] = data[pi](0, 2);
		system.partReorderBuffer[sortpi * 9 + 7] = data[pi](1, 2);
		system.partReorderBuffer[sortpi * 9 + 8] = data[pi](2, 2);
	}

#pragma omp parallel for schedule(static)
	for (int pi = 0; pi < system.partCount; pi++) {

		// double* ptr = &system.partReorderBuffer[pi * 9];
		// data[pi]	= Eigen::Map<Eigen::Matrix3d>(ptr, 3, 3);

		data[pi](0, 0) = system.partReorderBuffer[pi * 9 + 0];
		data[pi](1, 0) = system.partReorderBuffer[pi * 9 + 1];
		data[pi](2, 0) = system.partReorderBuffer[pi * 9 + 2];

		data[pi](0, 1) = system.partReorderBuffer[pi * 9 + 3];
		data[pi](1, 1) = system.partReorderBuffer[pi * 9 + 4];
		data[pi](2, 1) = system.partReorderBuffer[pi * 9 + 5];

		data[pi](0, 2) = system.partReorderBuffer[pi * 9 + 6];
		data[pi](1, 2) = system.partReorderBuffer[pi * 9 + 7];
		data[pi](2, 2) = system.partReorderBuffer[pi * 9 + 8];
	}
}

static void reorderAllParticleData(System& system)
{
	reorderParticleData(system.partPos, system);

	reorderParticleData(system.partVel, system);
	reorderParticleData(system.partB, system);

	reorderParticleData(system.partF_E, system);
	reorderParticleData(system.partF_P, system);
	reorderParticleData(system.partR_E, system);
	reorderParticleData(system.partJ_P, system);

	reorderParticleData(system.partColor, system);

#pragma omp parallel for schedule(static)
	for (int pi = 0; pi < system.partCount; pi++) {
		system.particles[pi].dataIndex = pi;
	}

	system.partDataSorted = true;
}

// update blocks with particle lists (begin and end index)
static void updateNodeBlockParticles(System& system)
{
	// update first and last particle
	int first = system.particles[0].blockIndex;
	int last  = system.particles[system.partCount - 1].blockIndex;

	system.blocks[first].partBegin = 0;
	system.blocks[first].partEnd   = system.partCount; // edge case - all parts in 1 block

	system.blocks[last].partEnd = system.partCount;

	// update all other nodes
#pragma omp parallel for
	for (int pi = 1; pi < system.partCount; pi++) {
		int left  = system.particles[pi - 1].blockIndex;
		int right = system.particles[pi + 0].blockIndex;

		// if this particle is this first in a node
		if (left != right) {
			// update nodes
			system.blocks[left].partEnd	= pi;
			system.blocks[right].partBegin = pi;
		}
	}
}

// computes the 27 node neighbors (including self) of the given node position
static NodeNeighbors computeNodeNeighbors(int x, int y, int z, std::array<Node, WORLD_NUM_NODES>& nodes)
{
	NodeNeighbors neighbors;

	for (int ki = 0; ki < WORLD_NUM_DIMENSIONS; ki++) {
		for (int kj = 0; kj < WORLD_NUM_DIMENSIONS; kj++) {
			for (int kk = 0; kk < WORLD_NUM_DIMENSIONS; kk++) {
				// compute neighbor position
				int kx = x + ki - 1;
				int ky = y + kj - 1;
				int kz = z + kk - 1;

				// index in neighbor node in local list
				int index = ki * 9 + kj * 3 + kk;

				// index of neighbor node in global list
				int kni = computeNodeIndex(kx, ky, kz);

				// add to neighbor list (or nullptr if out of bounds)
				neighbors[index] = isInBounds(kx, ky, kz) ? &nodes[kni] : nullptr;
			}
		}
	}

	return neighbors;
};