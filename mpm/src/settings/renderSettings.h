#pragma once

struct RenderSettings {
	enum ColorSetting {
		CLR_PARTICLE,
		CLR_ELASTIC,
		CLR_PLASTIC,
		CLR_INDEX,

		CLR_PARTITION
	};

	RenderSettings()
	{
		// default settings
		drawInverval = 1;
		writePNG	 = false;

		colorSetting = CLR_PARTICLE;

		pointSize   = 3.f;
		lineWidth   = 4.0f;
		vectorScale = 0.01f;

		showParticles		 = true;
		showParticleVelocity = false;

		showGrid		 = false;
		showActiveGrid   = false;
		showGridVelocity = false;

		showBoundary = true;

		colorChanged	  = true;
		visibilityChanged = true;
	};

	// draw rate
	int  drawInverval;
	bool writePNG;

	// color scheme
	ColorSetting colorSetting;

	// libigl settings
	float pointSize;
	float lineWidth;
	float vectorScale;

	// toggle visibility
	bool showParticles;
	bool showParticleVelocity;

	bool showGrid;
	bool showActiveGrid;
	bool showGridVelocity;
	bool showGridForce;

	bool showBoundary;

	bool colorChanged;
	bool visibilityChanged;
};