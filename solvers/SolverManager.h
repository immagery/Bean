#pragma once
#include "Solver.h"
#include "SolverChain.h"
#include "skeleton.h"
#include <vector>

using namespace std;

class SolverManager
{
public:
	vector<Solver*> solvers;

	vector<Point3d> computeSolvers(int frame, const vector<skeleton*>& skeletons);
	void addSolver(Solver *s) { 
		solvers.push_back(s);
	}

	SolverManager(void);
	~SolverManager(void);
};

