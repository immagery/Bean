#pragma once
#include "solverchain.h"
class SolverVerlet : public SolverChain {
public:
	vector<Eigen::Vector3d> restPositions;		// resting positions to compute the desired rest distance between links
	vector<Eigen::Vector3d> lastPositions;		// positions in last time
	vector<Eigen::Vector3d> currentPositions;
	vector<Eigen::Vector3d> idealPositions;

	pair<int, int> lookChain;
	Vector3d lookPoint;
	Vector3d lookVectorResting;
	Vector3d lookVector;


	double lastTime;					// compute timeStep 
	double stiffness;
	double g;
	double velocityDamping;

	Eigen::Vector3d positionAfterLast;

	SolverVerlet(void);
	~SolverVerlet(void);
	vector<pair<int,Eigen::Vector3d > > solveVerlet(double time, vector<SolverVerlet*>& verlets, int skID);
	vector<pair<int,Eigen::Vector3d > > solveCollisions(double time, vector<SolverVerlet*>& verlets, int skID);
	vector<pair<int,Eigen::Vector3d > > solve(double time) { return  vector<pair<int,Eigen::Vector3d > >(0);}
	void setPositions();
	Eigen::Vector3d idealRestPosition(int i);
};

