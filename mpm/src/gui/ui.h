#pragma once

#include "gui/parametersUI.h"
#include "gui/renderUI.h"
#include "gui/statsUI.h"

class RenderSettings;
class SimParameters;

class UI {
public:
	UI(RenderSettings& renderSettings, SimParameters& simParameters);

	void draw();

private:
	bool m_showRender;
	bool m_showParameters;
	bool m_showStats;

	// sub ui
	RenderUI	 m_renderUI;
	ParametersUI m_parametersUI;
	StatsUI		 m_statsUI;
};