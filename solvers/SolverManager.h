#pragma once
#include "Solver.h"
#include "SolverVerlet.h"
#include "skeleton.h"
#include "AdriViewer.h"
#include "Chain.h"
#include "Intelligence.h"
#include <vector>
#include <map>

using namespace std;

class SolverManager
{
public:
	SolverData* solverData;

	map<int, vector<Solver*> > solvers;
	map<int, Intelligence*> brains;
	map<int, double> previousLookAngles;
	map<int, int> numVueltas;
	
	vector<bool> verletEnabled;
	vector<bool> solversEnabled;
	bool twistCorrectionEnabled;

	Quaterniond myRot;

	int numTwisted, smoothingIterations;

	Quaterniond computeTwist (joint* jt, Vector3d nLook, Vector3d restUp);
	
	void newFrame (int frame) {

		solverData->time = frame * 1.0 / (solverData->fps);

		// Update dirtyness in solvers that require it
		for (int i = 0; i < solvers.size(); ++i) {
			vector<Solver*> vsolvers = solvers[i];
			for (int j = 0; j < vsolvers.size(); ++j) {
				vsolvers[j]->updateDirtyness();
				vsolvers[j]->time = frame * 1.0 / (solverData->fps);
			}
		}

		// AI actions
		for (int i = 0; i < brains.size(); ++i) {
			brains[i]->think();
		}
	}

	bool isDirty(int sk) {
		return solvers[sk][solvers[sk].size()-1]->dirtyFlag;
	}

	void draw(); 

	void update (int sk, skeleton* s);

	void addSkeleton(int id, skeleton* s) { 
		solvers[id] = vector<Solver*>();
		brains[id] = new Intelligence(id);
		previousLookAngles[id] = 0;
		numVueltas[id] = 0;
		solverData->skeletons.push_back(s);
	}

	void addSolver(Solver *s, int sk) {
		int lastSolver = solvers[sk].size() - 1;

		if (lastSolver >= 0) {
			if (solvers[sk][lastSolver]->outputs.size() > 1) 
				s->inputs.push_back(solvers[sk][lastSolver]->outputs[sk]);
			else 
				s->inputs.push_back(solvers[sk][lastSolver]->outputs[0]);
			Chain* c = new Chain();	c->positions.resize(20);
			s->outputs.push_back(c);
			solvers[sk][lastSolver]->children.push_back(s);
			s->fathers.push_back(solvers[sk][lastSolver]);
		} else {
			Chain* c = new Chain();	c->positions.resize(20);
			s->outputs.push_back(c);
		}

		solvers[sk].push_back(s);
		
	}

	SolverManager(void);
	~SolverManager(void);
};

