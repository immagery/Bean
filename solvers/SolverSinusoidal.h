#pragma once
#include "SolverChain.h"

// solver sinusoidal que trbaja sobre una dimension
class SolverSinusoidal : public SolverChain {
public:
	double amplitude;
	double freq;
	double phase;
	int dimension;		// 0=X, 1=Y, 2=Z
	vector<Eigen::Vector3d> restPositions;

	SolverSinusoidal(void);
	SolverSinusoidal(double a, double f, double ph);
	~SolverSinusoidal(void);
	vector<pair<int,Eigen::Quaternion<double> > > solve (double time);
	void setPositions() { 
		restPositions = vector<Eigen::Vector3d> (chain.size());
		for (int i = 0; i < chain.size(); ++i) {
			vcg::Point3d p = chain[i].first->getWorldPosition();
			restPositions[i] = Eigen::Vector3d(p.X(), p.Y(), p.Z());
		}
	}
};

