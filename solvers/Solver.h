#pragma once

#include <vector>
#include "Chain.h"
#include "SolverData.h"
#include "Node.h"

using namespace std;

class Solver : public node{
public:
	Solver(void) {
		inputs = vector<Chain*>();
		outputs = vector<Chain*>();	
		fathers = vector<Solver*>();	
		children = vector<Solver*>();	
		dirtyFlag = true;
	}
	Solver(SolverData* sd) {
		inputs = vector<Chain*>();
		outputs = vector<Chain*>();	
		fathers = vector<Solver*>();	
		children = vector<Solver*>();	
		dirtyFlag = true;
		data = sd;
	}

	~Solver(void) {}

	double time;
	vector<Chain*> inputs;
	vector<Chain*> outputs;
	vector<Solver*> fathers;
	vector<Solver*> children;

	int index1, index2, chainSize;

	SolverData* data;

	virtual void solve(SolverData* data) = 0;
	virtual void solve() = 0;

	virtual void updateDirtyness() {}

	virtual bool propagateDirtyness() {
		dirtyFlag = true;
		for (int i = 0; i < children.size(); ++i) children[i]->propagateDirtyness();
		return true;
	}

	virtual bool update() {
		if (dirtyFlag == false) return true;
		for (int i = 0; i < fathers.size(); ++i) fathers[i]->update();		// update all parents
		solve();															// solving process
		dirtyFlag = false;													// not dirty anymore
		return true;														// much clean
    }
};

