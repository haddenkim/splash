#pragma once

#include "settings/simParameters.h"
#include "solver/solverStep.h"
#include "state/system.h"

class Solver {
public:
	Solver()
	{
		currentStep = SolverStep::SOL_COMPLETE;
	}

	virtual ~Solver()
	{
	}

	// note: copy simParameters to keep thread-safe
	virtual void simulateOneTick(System& system, const SimParameters parameters) = 0;

	// for debugging
	virtual void advanceToStep(SolverStep targetStep, System& system, const SimParameters parameters) = 0;

	SolverStep currentStep;

	void advance(SolverStep& s)
	{
		using IntType = typename std::underlying_type<SolverStep>::type;

		// wrap to beginning if at end
		if (s == SolverStep::SOL_COMPLETE) {
			s = static_cast<SolverStep>(0);
		} else {
			// increment
			s = static_cast<SolverStep>(static_cast<IntType>(s) + 1);
		}
	};
};
