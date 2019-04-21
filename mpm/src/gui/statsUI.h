#pragma once

#include "settings/stats.h"

class StatsUI {
public:
	StatsUI(Stats& stats);

	void draw();

private:
	Stats& stats_;
};