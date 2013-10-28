#pragma once
#include "solverchain.h"
class SolverStatic :
	public SolverChain
{
public:
	vector<Quaternion<double> > staticAngles;

	SolverStatic(void);
	~SolverStatic(void);
	void setStatic();
	vector<pair<int,Quaternion<double> > > solve(double time);
};

