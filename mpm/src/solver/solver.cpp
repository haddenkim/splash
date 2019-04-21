#include "solver.h"
#include "kernels/interpolation.h"
#include <Eigen/Dense>
using namespace Eigen;

void Solver::reset()
{
	// reset bookkeeping
	activeNodes_.clear();
}

void Solver::advance(System& system, const SimParameters parameters, Stats& stats)
{
	// printf("solve step: %i\n", stats.stepCount);

	auto start = std::chrono::high_resolution_clock::now();

	resetGrid(system);
	clock(stats.timeReset, stats.totTimeReset, start);

	transferP2G(system, parameters);
	clock(stats.timeP2G, stats.totTimeP2G, start);

	computeGrid(system, parameters);
	clock(stats.timeGrid, stats.totTimeGrid, start);

	transferG2P(system, parameters);
	clock(stats.timeG2P, stats.totTimeG2P, start);

	computeParticle(system, parameters);
	clock(stats.timePart, stats.totTimePart, start);

	// log stats
	stats.simTime += parameters.timestep;
	stats.stepCount++;

	if (stats.trackEnergy) {
		computeTotalEnergy(stats, system);
	}

	additionalStats(stats, system);
}

void Solver::resetGrid(System& system)
{
	for (Node& node : system.nodes_) {
		node.mass = 0;
		node.vel.setZero();
		node.force.setZero();
	}
}

void Solver::transferP2G(System& system, const SimParameters& parameters)
{
	// compute common inertia-like tensor inverse (D_p)^-1 (paragraph after eq. 176)
	double commonDInvScalar = Interpolation::DInverseScalar(system.dx_);

	for (int pi = 0; pi < system.particles_.size(); pi++) {
		Particle& part = system.particles_[pi];

		// for convenience / optimization, pre-compute part constant contribution to node force VPFT
		part.VPFT = part.model->computeVolCauchyStress(part.vol0, part.F_E, part.R_E, part.J_P);

		// compute the particle's kernel
		Interpolation& kernel = part.kernel;
		kernel.compute(part.pos, system.dx_);

		// loop through kernel nodes
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {

					// compute grid frame coordinates of this iter's node
					int gridX = kernel.node0.x() + i;
					int gridY = kernel.node0.y() + j;
					int gridZ = kernel.node0.z() + k;

					// skip if out of bounds
					if (!system.isInBounds(gridX, gridY, gridZ)) {
						continue;
					}

					double   weight			= kernel.weight(i, j, k);
					Vector3d weightGradient = kernel.weightGradient(i, j, k);

					// reference to node
					Node& node = *system.getNode(gridX, gridY, gridZ);

					// accumulate mass (eq. 172)
					node.mass += weight * part.mass0;

					// compute vector from part to node in world frame (x_i - x_p)
					Vector3d vecPI = kernel.vecPI(i, j, k);

					// accumulate momentum (eq. 173)
					// stores in node velocity, next mpm step computes velocity from momentum
					node.vel += weight * part.mass0 * (part.vel + part.B * commonDInvScalar * vecPI);

					// internal stress force f_i (eq 189)
					node.force -= part.VPFT * weightGradient;
				}
			}
		}
	}
}

void Solver::computeGrid(System& system, const SimParameters& parameters)
{
	// loop through all nodes

	for (int ni = 0; ni < system.nodes_.size(); ni++) {
		// reference to node
		Node& node = system.nodes_[ni];

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

void Solver::transferG2P(System& system, const SimParameters& parameters)
{
	// loop through part
	for (Particle& part : system.particles_) {

		// reset velocity v_p and affine state B_p and velocity gradient ∇v_p
		part.vel.setZero();
		part.B.setZero();
		part.velGradient.setZero();

		// reference to kernel
		const Interpolation& kernel = part.kernel;

		// loop through kernel nodes
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {

					// compute grid frame coordinates of this iter's node
					int gridX = kernel.node0.x() + i;
					int gridY = kernel.node0.y() + j;
					int gridZ = kernel.node0.z() + k;

					// skip if out of bounds
					if (!system.isInBounds(gridX, gridY, gridZ)) {
						continue;
					}

					double   weight			= kernel.weight(i, j, k);
					Vector3d weightGradient = kernel.weightGradient(i, j, k);

					// reference to node
					Node& node = *system.getNode(gridX, gridY, gridZ);

					// accumulate velocity v_i (eq 175)
					part.vel += weight * node.vel;

					// compute vector from part to node in world frame (x_i - x_p)
					Vector3d vecPI = kernel.vecPI(i, j, k);

					// acculumate affine state B_i (eq 176)
					part.B += weight * node.vel * vecPI.transpose();

					// accumulate node's deformation update (eq 181) aka velocity gradient
					part.velGradient += node.vel * weightGradient.transpose();
				}
			}
		}
	}
}

void Solver::computeParticle(System& system, const SimParameters& parameters)
{
	for (Particle& part : system.particles_) {
		// update particle deformation gradient components
		part.model->updateDeformDecomp(part.F_E, part.R_E, part.F_P, part.J_P, part.velGradient, parameters.timestep);

		// Advection
		part.pos += parameters.timestep * part.vel;

		// process potential collision
		computeParticleCollision(part, system, parameters);
	}
}

void Solver::computeGridExtForces(Node& node, const System& system, const SimParameters& parameters)
{
	// TODO: implement other external forces

	// Gravity
	if (parameters.gravityEnabled) {
		node.force.y() -= parameters.gravityG * node.mass;
	}
}

void Solver::computeGridCollision(Node& node, const System& system, const SimParameters& parameters)
{
	// TODO: implement additional boundary conditions
	// TODO: consider more sophisticated collision response (ex. coefficients of restitution, friction)

	// enforce boundary conditions

	// non-elastic boundaries
	if (node.x < system.boundaryStart_	 // left wall
		|| node.x > system.boundaryEnd_	// right wall
		|| node.y > system.boundaryEnd_	// top wall (ceiling)
		|| node.z < system.boundaryStart_  // back wall
		|| node.z > system.boundaryEnd_) { // front wall
		node.vel.setZero();
	}

	// elastic boundary
	if (node.y < system.boundaryStart_) { // bottom wall (floor)
		node.vel.y() = std::max(0.0, node.vel.y());
	}
}

void Solver::computeParticleCollision(Particle& part, const System& system, const SimParameters& parameters)
{
	// TODO: implement additional boundary conditions
	// TODO: consider more sophisticated collision response (ex. coefficients of restitution, friction)

	// enforce boundary conditions

	// non-elastic boundaries
	if (part.pos.x() < system.boundaryStart_) // left wall
	{
		part.pos(0) = system.boundaryStart_;
		part.vel(0) = 0;
	}

	if (part.pos.x() > system.boundaryEnd_) // right wall
	{
		part.pos(0) = system.boundaryEnd_;
		part.vel(0) = 0;
	}

	if (part.pos.z() < system.boundaryStart_) // back wall
	{
		part.pos(2) = system.boundaryStart_;
		part.vel(2) = 0;
	}

	if (part.pos.z() > system.boundaryEnd_) // front wall
	{
		part.pos(2) = system.boundaryEnd_;
		part.vel(2) = 0;
	}

	if (part.pos.y() > system.boundaryEnd_) // top wall (ceiling)
	{
		part.pos(1) = system.boundaryEnd_;
		part.vel(1) = 0;
	}

	if (part.pos.y() < system.boundaryStart_) // bottom wall (floor)
	{
		part.pos(1) = system.boundaryStart_;
		part.vel(1) = std::max(0.0, part.vel(1));
	}
}

void Solver::clock(unsigned int& current, unsigned int& total, std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds>& start)
{
	auto end	  = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	start		  = end;

	current = duration.count();
	total += current;
}

void Solver::computeTotalEnergy(Stats& stats, const System& system)
{
	stats.totalKineticEnergy   = 0;
	stats.totalPotentialEnergy = 0;

	for (const Particle& part : system.particles_) {
		// kinetic energy
		stats.totalKineticEnergy += 0.5 * part.mass0 * part.vel.squaredNorm();
		stats.totalPotentialEnergy += part.model->computePotentialEnergy(part.F_E, part.R_E, part.F_P, part.vol0);
	}
}


void Solver::additionalStats(Stats& stats, const System& system)
{
}
