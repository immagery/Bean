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

	map<int, vector<Solver*> > solversChainRef;
	map<int, Intelligence*> brains;
	map<int, double> previousLookAngles;
	map<int, int> numVueltas;
	
	vector<bool> verletEnabled;
	vector<bool> solversEnabled;
	bool twistCorrectionEnabled;

	Quaterniond myRot;

	int numTwisted, smoothingIterations;
	
	SolverManager(void);
	~SolverManager(void);

	void newFrame (int frame); 
	void draw(); 

	bool isDirty(int sk); 
	void update (int sk, skeleton* s);

	void addSkeleton(int id, skeleton* s);
	void addSolver(Solver *s, int sk);

	Quaterniond computeTwist (joint* jt, Vector3d nLook, Vector3d restUp);
};

