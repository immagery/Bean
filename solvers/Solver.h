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

	void printName() {
		if (id == 0) printf("Init: ");
		if (id == 1) printf("Dir: ");
		if (id == 2) printf("Sin: ");
		if (id == 3) printf("Look: ");
		if (id == 4) printf("Verlet: ");
	}

	double time;
	vector<Chain*> inputs;
	vector<Chain*> outputs;
	vector<Solver*> fathers;
	vector<Solver*> children;

	int index1, index2, chainSize;
	int id;

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
		//printName();
		//printf("updating...\n");
		if (dirtyFlag == false) return true;
		//printf(" Updating parents...\n");
		clock_t start1 = clock();
		for (int i = 0; i < fathers.size(); ++i) fathers[i]->update();		// update all parents
		clock_t end1 = clock();
		clock_t start2 = clock();
		solve();															// solving process
		clock_t end2 = clock();
		//printName();
		//printf("Elapsed father update time: %f\n", timelapse(start1,end1));
		//printName();
		//printf("Elapsed self solve time: %f\n", timelapse(start2,end2));
		dirtyFlag = false;													// not dirty anymore
		return true;														// much clean
    }
};

