#pragma once
#include "solverchain.h"
class SolverStatic :
	public SolverChain
{
public:
	vector<Eigen::Vector3d > staticPositions;

	SolverStatic(void);
	~SolverStatic(void);
	void setStatic();
	vector<pair<int,Eigen::Vector3d > > solve(double time);
};

