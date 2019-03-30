#pragma once

#include <cmath>

double bSplineQuadratic(double x) // eq 123
{
	double aX = std::fabs(x);

	if (aX < 0.5) {
		return 0.75 - aX * aX;
	} else if (aX < 1.5) {
		return 0.5 * (1.5 - aX) * (1.5 - aX);
	} else {
		return 0.0;
	}
}

double bSplineQuadraticSlope(double x)
{
	double aX = std::fabs(x);

	if (aX < 0.5) {
		return -2.0 * x;
	} else if (aX < 1.5) {
		return x > 0.0 ? (x - 1.5) : (x + 1.5);
	} else {
		return 0.0;
	}

	return 0;
}