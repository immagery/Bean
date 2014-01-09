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

	SolverVerlet();
	~SolverVerlet();
	virtual void solve();
	void addSpringBetweenTwoJoints(int sk1, int sk2, int i1, int i2, double desiredDist, int springType, double deltaTime, int min1, int min2, double multiplier);
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



	double springForce (Vector3d p1, Vector3d p2, double springLength, double ks, double stiffness) {
		Vector3d distance = p1 - p2;
		if (distance.norm() > 0) {
			double diff = distance.norm() - springLength;
			Vector3d delta = distance / distance.norm() * ks;
			return 0;
		}
	}

	void attractingForce (int ip, int i, Vector3d restDistance, double deltaTime) {
		Vector3d currentPoint = currentPositions[ip][i];
		Vector3d idealPoint = idealPositions[ip][i];
		Vector3d currentDist = currentPoint - idealPoint;
		if (currentDist.norm() > 0) {			// avoid dividing by 0
			double diff = currentDist.norm() - restDistance.norm();
			Vector3d delta1 = currentDist / currentDist.norm() * posS * diff;
			Vector3d vel1 = (currentPositions[ip][i] - lastPositions[ip][i]);
			Vector3d rigid = delta1;
			double v = (currentDist.normalized()).dot(vel1.normalized());
			if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
			Vector3d damp1 = currentDist / currentDist.norm() * posD * v;
			Vector3d inc = (delta1+damp1+rigid*positioningStrengths[i])*(posStiff)*deltaTime;
			currentPositions[ip][i] -= inc;
		}
	}
};

