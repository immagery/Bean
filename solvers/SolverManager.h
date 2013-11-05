#pragma once
#include "Solver.h"
#include "SolverChain.h"
#include "SolverVerlet.h"
#include "skeleton.h"
#include <vector>
#include <map>

using namespace std;

class SolverManager
{
public:
	map<int, vector<Solver*> > solvers;
	map<int, vector<Solver*> > postSolvers;
	vector<SolverVerlet*> verlets;

	bool dumpVectors;
	
	vector<bool> verletEnabled;
	vector<bool> solversEnabled;

	double dividingBaseFactor;

	vector<vcg::Quaternion<double> > computeSolvers(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk);
	vector<vcg::Quaternion<double> > computeVerlet(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk);
	vector<vcg::Quaternion<double> > computePostSolvers(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk);

	void addSkeleton(int id) { 
		solvers[id] = vector<Solver*>();
		postSolvers[id] = vector<Solver*>();
		verlets.push_back(new SolverVerlet());
		verletEnabled.push_back(true);
		solversEnabled.push_back(false);
	}

	void addSolver(Solver *s, int i) { 
		solvers[i].push_back(s);
	}

	void addPostSolver(Solver *s, int i) { 
		postSolvers[i].push_back(s);
	}

	SolverManager(void);
	~SolverManager(void);
};

