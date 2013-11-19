#pragma once
#include "SolverChain.h"

class SolverSinusoidal : public SolverChain {
public:

	vector<Eigen::Vector3d> restPositions;

	double lastTime;

	// Wave parameters
	double amplitude;
	double freq;
	double phase;
	int dimension;		// 0=X, 1=Y, 2=Z
	double longitude;

	// Constructors and destructors
	SolverSinusoidal(void) {}
	SolverSinusoidal(double a, double f, double ph) : amplitude(a), freq(f), phase(ph) {}
	~SolverSinusoidal(void) {}
	void setPositions();

	// Solving
	vector<pair<int,Eigen::Vector3d > > solve (double time);
};

