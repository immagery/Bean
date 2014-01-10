#pragma once
#include "Solver.h"
#include "skeleton.h"

class SolverVerlet : public Solver {
public:
	vector<vector<Vector3d> > restPositions;		// resting positions to compute the desired rest distance between links
	vector<vector<Vector3d> > lastPositions;					// positions in last time
	vector<vector<Vector3d> > currentPositions;
	vector<vector<Vector3d> > idealPositions;

	double lastTime, g, velocityDamping, fps;
	bool hasGravity;
	bool hasRigid;
	bool lookSolver;		// para hacer casos especiales

	// Spring parameters: Strength, Damping, Stiffness
	double distS, distD, distStiff;		// springs between nodes
	double posS, posD, posStiff;		// springs between nodes and ideal positions
	double colS, colD, colStiff;		// dynamic springs to avoid collisions
	double rigidS, rigidD, rigidStiff;
	double slider;
	vector<double> positioningStrengths;
	vector<double> rigidnessStrengths;

	SolverVerlet* nextVerlet;		// points to next verlet in order to send current and last positions

	SolverVerlet();
	~SolverVerlet();
	virtual void solve();
	void addSpringBetweenTwoJoints(int sk1, int sk2, int i1, int i2, double desiredDist, int springType, double deltaTime, int min1, int min2, double multiplier);
	void addSpringBetweenTwoJoints3D(int sk1, int sk2, int i1, int i2, double desiredDist, int springType, double deltaTime, int min1, int min2, double multiplier);
	void addSpringToPoint (int sk1, int i1, double desiredDist, Vector3d p, int springType, double deltaTime, int min1, double multiplier);
	void solve2(double ttime);
	void solve3(double ttime);
	
	vector<pair<int,Vector3d> > SolverVerlet::solveVerlet(double time, vector<SolverVerlet*>& verlets, int skID);
	void solveVerlet2 (double time, SolverData* data);
	
	virtual void solve(double time) {}

	void addSkeleton(skeleton* s, Chain* c) {
		chainSize = index2 - index1 + 1;
		int nextSK = restPositions.size();
		restPositions.push_back(vector<Vector3d>(chainSize));
		lastPositions.push_back(vector<Vector3d>(chainSize));
		currentPositions.push_back(vector<Vector3d>(chainSize));
		idealPositions.push_back(vector<Vector3d>(chainSize));
		positioningStrengths = rigidnessStrengths = vector<double> (chainSize, 0);

		for (int i = 0; i < chainSize; ++i) {
			currentPositions[nextSK][i] = lastPositions[nextSK][i] = restPositions[nextSK][i] = c->positions[i];
			//positioningStrengths[i] = (1 - 0.03*i);
		}
	}

	void setPositions(skeleton* s) {

		chainSize = index2 - index1 + 1;

		restPositions.resize(inputs.size());
		lastPositions.resize(inputs.size());
		currentPositions.resize(inputs.size());
		idealPositions.resize(inputs.size());
		positioningStrengths = vector<double> (chainSize, 1);

		for (int ip = 0; ip < inputs.size(); ++ip) {
			restPositions[ip].resize(chainSize);
			lastPositions[ip].resize(chainSize);
			currentPositions[ip].resize(chainSize);
			idealPositions[ip].resize(chainSize);

			for (int i = 0; i < chainSize; ++i) {
				currentPositions[0][i] = lastPositions[0][i] = restPositions[0][i] = s->joints[i]->translation;
				positioningStrengths[i] = (1 - 0.03*i);
			}
		}
	}

	void solve(SolverData* data) {
		if (hasGravity) g = data->gravity;
		else g = 0;
		time = data->time;
		fps = data->fps;
		solve();
	}

	virtual void updateDirtyness() {
		dirtyFlag = true;
		propagateDirtyness();
	}

};

