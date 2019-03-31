#pragma once

enum SolverStep {
	SOL_RESET,
	SOL_LINKS,
	SOL_P2G,
	SOL_DOF,
	SOL_FORCE,
	SOL_VEL,
	SOL_G2P,
	SOL_ADVECT,
	SOL_COMPLETE
};

static const char* SolverStepName[] = {
	"SOL_RESET",
	"SOL_LINKS",
	"SOL_P2G",
	"SOL_DOF",
	"SOL_FORCE",
	"SOL_VEL",
	"SOL_G2P",
	"SOL_ADVECT",
	"SOL_COMPLETE"
};