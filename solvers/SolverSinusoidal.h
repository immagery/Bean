#pragma once
#include "SolverChain.h"

// solver sinusoidal que trbaja sobre una dimension
class SolverSinusoidal : public SolverChain {
public:
	double amplitude;
	double freq;
	double phase;
	int dimension;		// 0=X, 1=Y, 2=Z

	SolverSinusoidal(void);
	SolverSinusoidal(double a, double f, double ph);
	~SolverSinusoidal(void);
	vector<pair<int,Point3d> > solve (double time);
};

