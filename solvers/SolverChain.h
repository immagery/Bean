#pragma once
#include "skeleton.h"
#include "Solver.h"
#include <vector>

// A SolverChain works on a set of bones
class SolverChain : public Solver {
public:
	vector<pair<joint*, int> > chain;
	vector<Point3d> distances;

	SolverChain(void);
	~SolverChain(void);
	void addJoint (joint* j, int index) { 
		chain.push_back(pair<joint*, int> (j, index));
		if (chain.size() > 1) distances.push_back(chain.back().first->worldPosition - chain[chain.size()-2].first->worldPosition);
	}
	virtual vector<pair<int,Point3d> > solve(double time) = 0;
};

