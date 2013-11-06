#pragma once
#include "solverchain.h"
class SolverVerlet : public SolverChain {
public:
	vector<Eigen::Vector3d> restPositions;		// resting positions to compute the desired rest distance between links
	vector<Eigen::Vector3d> lastPositions;		// positions in last time
	vector<Eigen::Vector3d> currentPositions;	
	vector<vector<Eigen::Vector3d> > bakedPositions;

	double lastTime;					// compute timeStep 
	double stiffness;
	double g;
	double velocityDamping;

	Point3d positionAfterLast;

	SolverVerlet(void);
	~SolverVerlet(void);
	vector<pair<int,Eigen::Vector3d > > solveVerlet(double time, vector<SolverVerlet*>& verlets, int skID);
	vector<pair<int,Eigen::Quaternion<double> > > solve(double time) { return  vector<pair<int,Eigen::Quaternion<double> > >(0);}
	void setPositions();
	//void solve(int frame, double animationPeriod, vector<skeleton*> skeletons, int skID);
	void bake(int maxFrames, double deltaTimePerFrame);
	Eigen::Vector3d idealRestPosition(int i);
};

