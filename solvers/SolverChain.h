#pragma once
#include "skeleton.h"
#include "Solver.h"
#include <vector>

using namespace std;
using namespace Eigen;

// A SolverChain works on a set of bones
class SolverChain : public Solver {
public:
	vector<pair<int, Vector3d> > chain;

	SolverChain(void);
	~SolverChain(void);
	void addJoint (int index, Eigen::Vector3d pos) { 
		chain.push_back(pair<int, Vector3d> (index, pos));
	}
	virtual vector<pair<int,Eigen::Vector3d > > solve(double time) = 0;
};

