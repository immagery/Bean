#pragma once
#include "SolverChain.h"

// solver sinusoidal que trbaja sobre una dimension
class SolverSinusoidal : public SolverChain {
public:
	double amplitude;
	double freq;
	double phase;
	int dimension;		// 0=X, 1=Y, 2=Z
	vector<Point3d> restPositions;

	SolverSinusoidal(void);
	SolverSinusoidal(double a, double f, double ph);
	~SolverSinusoidal(void);
	vector<pair<int,Quaternion<double> > > solve (double time);
	void setPositions() { 
		restPositions = vector<Point3d> (chain.size());
		for (int i = 0; i < chain.size(); ++i) restPositions[i] = chain[i].first->getWorldPosition();
	}
};

