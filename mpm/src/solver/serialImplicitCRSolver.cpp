#include "serialImplicitCRSolver.h"

using namespace Eigen;

void SerialImplicitCRSolver::resetGrid(System& system)
{
}

void SerialImplicitCRSolver::computeGrid(System& system, const SimParameters& parameters)
{
    updateActiveNodeList(system);

    // setup CR vectors
    int dof = activeNodes_.size() * 3;
    VectorXd CR_x(dof);
    VectorXd CR_r(dof);
    VectorXd CR_p(dof);
    VectorXd CR_Ap(dof);
    VectorXd CR_Ar(dof);

    // set initial guess to current velocity (para above 201)
    for(int ni = 0; ni < activeNodes_.size(); ni++)
    {
        CR_x.segment<3>(3* ni) = activeNodes_[ni]->vel;
    }

    
    

    // goal is to compute next node velocities

    // basic Conjugate Residual method:

    // compute the initial CR values
    //   CR x_0 = velocity guess
    //     either guess zero or current velocity or compute explicit next velocity
    //   CR r_0 residuals
    //   CR p_0 
     
    // compute CR r_0 and p_0

    // CR loop
    // while (tolerance is not met or max iters is not met)
    //   compute CR alpha
    //   compute CR 

}

void SerialImplicitCRSolver::updateActiveNodeList(System& system)
{
	// produce list of active nodes for reduced DoF solves
	for (int i = 0; i < system.gridSize_; i++) {
		for (int j = 0; j < system.gridSize_; j++) {
			for (int k = 0; k < system.gridSize_; k++) {

				// reference to node
				Node& node = system.nodes_[i][j][k];

				// skip if node has no mass (aka no particles nearby)
				if (node.mass == 0) {
					continue;
				}

				// compute velocity from momentum / mass
				node.vel /= node.mass;

				// update node record
				node.activeNodeIndex = activeNodes_.size();

				// add to active node list
				activeNodes_.push_back(&node);
				activeNodeIndex_.emplace_back(Vector3i(i, j, k));
			}
		}
	}
}