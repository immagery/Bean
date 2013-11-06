#pragma once
#include "solverchain.h"

class SolverLook :
	public SolverChain
{
public:
	Eigen::Vector3d lookVector;
	Eigen::Vector3d lookPoint;

	SolverLook(void);
	~SolverLook(void);
	vector<pair<int,Eigen::Quaternion<double> > > solve(double time);
};

