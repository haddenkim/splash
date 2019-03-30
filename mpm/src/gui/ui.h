#pragma once

#include "gui/parametersUI.h"
#include "gui/renderUI.h"
#include "gui/statsUI.h"
#include "gui/systemUI.h"

class RenderSettings;
class SimParameters;
class System;

class UI {
public:
	UI(RenderSettings& renderSettings, SimParameters& simParameters, const System& system);

	void draw();

private:
	bool showRender_;
	bool showParameters_;
	bool showStats_;
	bool showSystem_;

	// sub ui
	RenderUI	 renderUI_;
	ParametersUI parametersUI_;
	StatsUI		 statsUI_;
	SystemUI	 systemUI_;
};