#pragma once

#include "settings/stats.h"

class StatsUI {
public:
	StatsUI(const Stats& stats);

	void draw();

private:
	const Stats& stats_;
};