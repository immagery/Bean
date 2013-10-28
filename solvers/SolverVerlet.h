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
	double velocityDamping;

	Point3d positionAfterLast;

	SolverVerlet(void);
	~SolverVerlet(void);
	vector<pair<int,Point3d > > solveVerlet(double time, vector<SolverVerlet*>& verlets, int skID);
	vector<pair<int,Quaternion<double> > > solve(double time) { return  vector<pair<int,Quaternion<double> > >(0);}
	void setPositions();
	//void solve(int frame, double animationPeriod, vector<skeleton*> skeletons, int skID);
	void bake(int maxFrames, double deltaTimePerFrame);
	Point3d idealRestPosition(int i);
};

