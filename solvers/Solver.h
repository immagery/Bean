#pragma once

#include <vector>
#include "Chain.h"
#include "SolverData.h"
#include "Node.h"

using namespace std;

class Solver : public node{
public:
	Solver(int _id) 
	{
		inputs = vector<Chain*>();
		outputs = vector<Chain*>();	
		fathers = vector<Solver*>();	
		children = vector<Solver*>();	
		dirtyFlag = true;
		id = _id;
	}

	Solver(SolverData* sd) 
	{
		inputs = vector<Chain*>();
		outputs = vector<Chain*>();	
		fathers = vector<Solver*>();	
		children = vector<Solver*>();	
		dirtyFlag = true;
		data = sd;
	}

	~Solver(void) 
	{
		// We only remove the pointers,
		// The data will be cleaned in other places.
		inputs.clear();
		outputs.clear();
		fathers.clear();
		children.clear();
		data = NULL;
	}

	void printName() 
	{
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

	virtual bool propagateDirtyness() 
	{
		dirtyFlag = true;

		// Propagate the dirty flag to all the children
		for (int i = 0; i < children.size(); ++i) 
			children[i]->propagateDirtyness();

		return true;
	}

	virtual bool update() 
	{
		if (dirtyFlag == false) return true;

		// Test if all the fathers are clean
		bool fathersUpdated = true;
		for (int i = 0; i < fathers.size(); ++i) 
			fathersUpdated &= fathers[i]->update();								// update all parents

		// If all the fathers are clean this node can be solved
		if(fathersUpdated)
		{
			solve();															// solving process
			dirtyFlag = false;													// not dirty anymore
			return true;
		}
		else return false;														// much clean
    }
};

