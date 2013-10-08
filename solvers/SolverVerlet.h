#pragma once
#include "solverchain.h"
class SolverVerlet : public SolverChain {
public:
	vector<Point3d> restPositions;		// resting positions to compute the desired rest distance between links
	vector<Point3d> lastPositions;		// positions in last time
	vector<Point3d> currentPositions;	
	vector<vector<Point3d> > bakedPositions;

	double lastTime;					// compute timeStep 
	double stiffness;
	double g;

	SolverVerlet(void);
	~SolverVerlet(void);
	vector<pair<int,Point3d> > solve(double time);
	void setPositions();
	void bake(int maxFrames, double deltaTimePerFrame);
};

