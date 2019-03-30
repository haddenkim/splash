#pragma once

#include "settings/simParameters.h"

class ParametersUI {
public:
	ParametersUI(SimParameters& simParameters);

	void draw();

private:
	SimParameters& simParameters_;
};