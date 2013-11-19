#pragma once
#include "Solver.h"
#include "SolverChain.h"
#include "SolverVerlet.h"
#include "skeleton.h"
#include "Chain.h"
#include <vector>
#include <map>

using namespace std;

class SolverManager
{
public:
	map<int, Chain*> restChains;
	map<int, Chain*> currentChains;
	map<int, Chain*> idealChains;
	map<int, vector<Solver*> > solvers;
	map<int, vector<Solver*> > postSolvers;
	vector<SolverVerlet*> verlets;

	bool dumpVectors;
	
	vector<bool> verletEnabled;
	vector<bool> solversEnabled;

	double dividingBaseFactor;

	vector<Eigen::Quaternion<double> > solve (int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk);

	void addSkeleton(int id, skeleton* s) { 
		Chain* c = new Chain();
		Chain* ideal = new Chain();
		Chain* rest = new Chain();
		for (int i = 0; i < s->joints.size(); ++i) {
			if (i <= s->joints.size() - 6) {
				c->positions.push_back(s->joints[i]->getWorldPosition());
				ideal->positions.push_back(s->joints[i]->getWorldPosition());
				rest->positions.push_back(s->joints[i]->getWorldPosition());
			} else {
				Vector3d wpos = s->joints[i-1]->getWorldPosition();
				Vector3d wpos2 = s->joints[i-2]->getWorldPosition();
				Vector3d increment = wpos - wpos2;
				increment *= 5;	//	increment += Vector3d(0,10,0);
				increment = Vector3d(0,0,20);
				c->positions.push_back(wpos + increment);
				ideal->positions.push_back(wpos + increment);
				rest->positions.push_back(wpos + increment);
				break;
			}
		}

		currentChains[id] = c;
		idealChains[id] = ideal;
		restChains[id] = rest;
		solvers[id] = vector<Solver*>();
		postSolvers[id] = vector<Solver*>();
		verlets.push_back(new SolverVerlet());
		verletEnabled.push_back(true);
		solversEnabled.push_back(true);
	}

	void addSolver(Solver *s, int i) { 
		solvers[i].push_back(s);
	}

	void addPostSolver(Solver *s, int i) { 
		postSolvers[i].push_back(s);
	}

	void addJointToSolver (int skID, int jointID, SolverChain* s) {
		s->addJoint(jointID, currentChains[skID]->positions[jointID]);
	}

	SolverManager(void);
	~SolverManager(void);
};

