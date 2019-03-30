#pragma once

#include "state/system.h"

class SystemUI {
public:
	SystemUI(const System& system);

	void draw();

private:
	const System& system_;

	bool showParticles_;
	bool showNodes_;
	bool showLinks_;

	void displayParticleData();
	void displayNodeData();
	void displayLinkData();
};