#pragma once
#include "SolverChain.h"

class SolverSinusoidal : public SolverChain {
public:

	// Wave parameters
	double amplitude;
	double freq;
	double phase;
	int dimension;		// 0=X, 1=Y, 2=Z

	// Constructors and destructors
	SolverSinusoidal(void) {}
	SolverSinusoidal(double a, double f, double ph) : amplitude(a), freq(f), phase(ph) {}
	~SolverSinusoidal(void) {}

	// Solving
	vector<pair<int,Eigen::Quaternion<double> > > solve (double time);
};

