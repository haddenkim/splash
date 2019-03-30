#pragma once

struct RenderSettings {
	RenderSettings()
	{
		// default settings
		pointSize = 10.f;
        lineWidth = 1.0f;

		showParticles		 = true;
		showParticleVelocity = false;

		showGrid		 = false;
		showActiveGrid   = false;
		showGridVelocity = false;

		showFloor = true;
	};

	// libigl settings
	float pointSize;
    float lineWidth;

	// toggle visibility
	bool showParticles;
	bool showParticleVelocity;

	bool showGrid;
	bool showActiveGrid;
	bool showGridVelocity;

	bool showFloor;
};