#include "solver/solverOmpScatter.h"
#include "kernels/interpolation.h"
#include "settings/simParameters.h"
#include "state/system.h"
#include "state/systemHelpers.h"
#include <algorithm>
#include <omp.h>

using namespace Eigen;

void SolverOmpScatter::resetGrid(System& system)
{
#pragma omp parallel for
	for (int ni = 0; ni < WORLD_NUM_NODES; ni++) {
		Node& node = system.nodes[ni];

		// skip if node has no mass (aka no particles nearby)
		if (node.mass == 0) {
			continue;
		}

		node.vel.setZero();
		node.force.setZero();
		node.mass = 0;
	}

#pragma omp parallel for
	for (int bi = 0; bi < WORLD_NUM_BLOCKS; bi++) {
		system.blocks[bi].partBegin = 0;
		system.blocks[bi].partEnd   = 0;
	}
}

void SolverOmpScatter::transferP2G(System& system, const SimParameters& parameters)
{

	sortParticleIndices(system);
	updateNodeBlockParticles(system);

	// for each set of alternating nodes blocks
	for (int si = 0; si < WORLD_NUM_SETS; si++) {
		auto& set = system.sets[si];

		// for each block in this set
#pragma omp parallel for schedule(dynamic)
		for (int sbi = 0; sbi < SET_NUM_BLOCKS; sbi++) {
			// get global block index
			int bi = set[sbi];

			// p2g this block
			p2gSingleBlock(system, bi);
		}
	}

	//////////////////////
	/* sort particles into blocks */
	//   sort particles by containing cell morton id
	//   (possibly sort by containing block morton id)
	//   for each block
	//     find and store index of first particle
	//     (possibly parallel compare to right -> if diff, store right's index and node )

	/* P2G */
	// for each set of nodes (in top level subdivision) ( n = 8)
	//   par for every block in set
	//     for every particle in block
	//       particle -> 27 node transfers
	//
	// barrier sync after every set
}

void SolverOmpScatter::p2gSingleBlock(System& system, int bi)
{
	// get block
	const NodeBlock& block = system.blocks[bi];

	// loop through particles in block
	for (int sortpi = block.partBegin; sortpi < block.partEnd; sortpi++) {
		// index of particle's data
		int pi = system.particles[sortpi].dataIndex;

		// for convenience / optimization, pre-compute part constant contribution to node force VPFT
		Matrix3d partVPFT = system.partModel[pi]->computeVolCauchyStress();
		
		// pre-compute particle's interpolation
		auto&		  partPos = system.partPos[pi];
		Interpolation kernel(partPos);

		// index of particle's containing node
		int ni = computeNodeIndex(partPos);

		// loop through nodes in kernel
		for (Node* kernelNode : system.neighbors[ni]) {
			if (kernelNode == nullptr) {
				continue;
			}

			// kernel node's position relative to particle's kernel
			Vector3i ijk = kernelNode->pos - kernel.node0;

			// retrieve node-particle data
			double   commonDInvScalar = Interpolation::DInverseScalar();
			double   weight			  = kernel.weight(ijk.x(), ijk.y(), ijk.z());
			Vector3d weightGradient   = kernel.weightGradient(ijk.x(), ijk.y(), ijk.z());
			Vector3d vecPI			  = kernel.vecPI(ijk.x(), ijk.y(), ijk.z());

			// accumulate mass (eq. 172)
			kernelNode->mass += weight * system.partMass[pi];

			// accumulate momentum (eq. 173)
			// stores in node velocity, next mpm step computes velocity from momentum
			kernelNode->vel += weight * system.partMass[pi] * (system.partVel[pi] + system.partB[pi] * commonDInvScalar * vecPI);

			// internal stress force f_i (eq 189)
			kernelNode->force -= partVPFT * weightGradient;
		}
	}
}

void SolverOmpScatter::computeGrid(System& system, const SimParameters& parameters)
{
	// loop through all active nodes
#pragma omp parallel for
	for (int ni = 0; ni < WORLD_NUM_NODES; ni++) {
		Node& node = system.nodes[ni];

		// skip if node has no mass (aka no particles nearby)
		if (node.mass == 0) {
			continue;
		}

		// compute velocity from momentum / mass
		node.vel /= node.mass;

		// apply external forces
		computeGridExtForces(node, system, parameters);

		// update grid velocities
		node.vel = node.vel + parameters.timestep * node.force / node.mass;

		// process potential collision
		computeGridCollision(node, system, parameters);
	}
}

void SolverOmpScatter::transferG2P(System& system, const SimParameters& parameters)
{
	// loop through part
#pragma omp parallel for schedule(dynamic)
	for (int sortpi = 0; sortpi < system.partCount; sortpi++) {
		// index of particle's data
		int pi = system.particles[sortpi].dataIndex;

		// reset velocity v_p and affine state B_p and velocity gradient ∇v_p
		system.partVel[pi].setZero();
		system.partB[pi].setZero();
		system.partVelGrad[pi].setZero();

		// pre-compute particle's interpolation
		auto&		  partPos = system.partPos[pi];
		Interpolation kernel(partPos);

		// index of particle's containing node
		int ni = computeNodeIndex(partPos);

		// loop through nodes in kernel
		for (Node* kernelNode : system.neighbors[ni]) {
			if (kernelNode == nullptr) {
				continue;
			}

			// kernel node's position relative to particle's kernel
			Vector3i ijk = kernelNode->pos - kernel.node0;

			// retrieve node-particle data
			double   weight			= kernel.weight(ijk.x(), ijk.y(), ijk.z());
			Vector3d weightGradient = kernel.weightGradient(ijk.x(), ijk.y(), ijk.z());
			Vector3d vecPI			= kernel.vecPI(ijk.x(), ijk.y(), ijk.z());

			// accumulate velocity v_i (eq 175)
			system.partVel[pi] += weight * kernelNode->vel;

			// acculumate affine state B_i (eq 176)
			system.partB[pi] += weight * kernelNode->vel * vecPI.transpose();

			// accumulate node's deformation update (eq 181)
			system.partVelGrad[pi] += kernelNode->vel * weightGradient.transpose();
		}
	}
}

void SolverOmpScatter::computeParticle(System& system, const SimParameters& parameters)
{
#pragma omp parallel for schedule(static)
	for (int pi = 0; pi < system.partCount; pi++) {

		// update particle deformation gradient components
		system.partModel[pi]->updateDeformation(system.partVelGrad[pi], parameters.timestep);

		// for convenience
		auto& pos = system.partPos[pi];
		auto& vel = system.partVel[pi];

		// Advection
		pos += parameters.timestep * vel;

		// process potential collision
		computeParticleCollision(pos, vel, system, parameters);
	}
}
