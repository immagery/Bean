#pragma once
#include "solverchain.h"
class SolverStatic :
	public SolverChain
{
public:
	vector<Point3d> staticAngles;

	SolverStatic(void);
	~SolverStatic(void);
	void setStatic();
	vector<pair<int,Point3d> > solve(double time);
};

