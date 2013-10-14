#pragma once
#include "Solver.h"
#include "SolverChain.h"
#include "SolverVerlet.h"
#include "skeleton.h"
#include <vector>

using namespace std;

class SolverManager
{
public:
	vector<Solver*> solvers;
	SolverVerlet* verlet;
	bool hasVerlet;

	vector<Point3d> computeSolvers(int frame, int animationPeriod, const vector<skeleton*>& skeletons);
	vector<Point3d> computeVerlet(int frame, int animationPeriod, const vector<skeleton*>& skeletons);
	void addSolver(Solver *s) { 
		solvers.push_back(s);
	}

	SolverManager(void);
	~SolverManager(void);
};

