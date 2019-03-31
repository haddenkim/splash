#pragma once

#include "gui/debugUI.h"
#include "gui/parametersUI.h"
#include "gui/renderUI.h"
#include "gui/statsUI.h"
#include "gui/systemUI.h"

class RenderSettings;
class SimParameters;
class System;

class UI {
public:
	UI(Solver*		   solver,
	   RenderSettings& renderSettings,
	   SimParameters&  simParameters,
	   System&		   system,
	   Stats&		   stats);

	void draw();

private:
	bool showDebug_;
	bool showRender_;
	bool showParameters_;
	bool showStats_;
	bool showSystem_;

	// sub ui
	DebugUI		 debugUI_;
	RenderUI	 renderUI_;
	ParametersUI parametersUI_;
	StatsUI		 statsUI_;
	SystemUI	 systemUI_;
};