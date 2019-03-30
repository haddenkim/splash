#pragma once

#include "settings/renderSettings.h"

class RenderUI {
public:
	RenderUI(RenderSettings& renderSettings);

	void draw();

private:
	RenderSettings& m_renderSettings;
};
