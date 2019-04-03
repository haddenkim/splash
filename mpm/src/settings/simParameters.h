#pragma once

struct SimParameters {
	SimParameters()
	{
		// default settings
		timestep =  1e-4;

		gravityEnabled = true;
		gravityG	   = 200;

		criticalCompression = 2.5e-2;
		criticalStretch		= 7.5e-3;
		hardening			= 10.0;
		E					= 1e4;
		nu					= 0.2;

		// compute and store initial mu and lambda (eq 47)
		mu0		= E / (2 * (1 + nu));
		lambda0 = E * nu / ((1 + nu) * (1 - 2 * nu));
	};

	float timestep;

	bool  gravityEnabled;
	float gravityG;

	// Snow material properties
	double criticalCompression; // θ_c
	double criticalStretch;		// θ_s
	double hardening;			// ξ
	double E;					// Young's Modulus
	double nu;					// Poisson ratio
	double mu0;					// μ shear modulus
	double lambda0;				// λ Lame's first parameter
};