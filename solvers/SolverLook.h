#pragma once
#include "solverchain.h"

class SolverLook :
	public SolverChain
{
public:
	Point3d lookVector;
	Point3d lookPoint;

	SolverLook(void);
	~SolverLook(void);
	vector<pair<int,Eigen::Quaternion<double> > > solve(double time);
};

