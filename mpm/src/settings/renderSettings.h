#pragma once

struct RenderSettings {
	RenderSettings()
	{
		// default settings
		drawInverval = 1;

		pointSize = 3.f;
		lineWidth = 1.0f;

		showParticles		 = true;
		showParticleVelocity = false;

		showGrid		 = false;
		showActiveGrid   = false;
		showGridVelocity = false;

		showBoundary = true;
	};

	// draw rate
	int drawInverval;

	// libigl settings
	float pointSize;
	float lineWidth;

	// toggle visibility
	bool showParticles;
	bool showParticleVelocity;

	bool showGrid;
	bool showActiveGrid;
	bool showGridVelocity;
	bool showGridForce;

	bool showBoundary;
};