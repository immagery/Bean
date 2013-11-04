#pragma once
#include "solverchain.h"
class SolverStatic :
	public SolverChain
{
public:
	vector<Eigen::Quaternion<double> > staticAngles;

	SolverStatic(void);
	~SolverStatic(void);
	void setStatic();
	vector<pair<int,Eigen::Quaternion<double> > > solve(double time);
};

