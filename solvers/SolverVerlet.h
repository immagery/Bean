#pragma once
#include "Solver.h"
#include "skeleton.h"

// todo: poner un flag para poder hacer c�lculos sin sobreescribir durante un frame
// usar para no cambiar el segmento entre pasos de relajaci�n

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

	int ccc;
	bool printStuff;
	bool updateFlag;

	// Attack curve
	vector<vector<Vector3d> > attackCurves;
	vector<vector<Vector3d> > curves;

	// Spring parameters: Strength, Damping, Stiffness
	double distS, distD, distStiff;		// springs between nodes
	double posS, posD, posStiff;		// springs between nodes and ideal positions
	double colS, colD, colStiff;		// dynamic springs to avoid collisions
	double rigidS, rigidD, rigidStiff;
	double slider;
	vector<double> positioningStrengths;
	vector<double> rigidnessStrengths;

	int desvinculado;
	int minVinculado;
	vector<int> lastMinIndexes;
	bool attackFlag;
	Vector3d lookPoint;
	bool moving;
	
	SolverVerlet* nextVerlet;		// points to next verlet in order to send current and last positions

	SolverVerlet();
	~SolverVerlet();
	virtual void solve();
	void addSpringBetweenTwoJoints(int sk1, int sk2, int i1, int i2, double desiredDist, int springType, double deltaTime, int min1, int min2, double multiplier);
	void addSpringBetweenTwoJoints3D(int sk1, int sk2, int i1, int i2, double desiredDist, int springType, double deltaTime, int min1, int min2, double multiplier);
	void addSpringToPoint (int sk1, int i1, double desiredDist, Vector3d p, int springType, double deltaTime, int min1, double multiplier);
	void solve2(double ttime);
	void solve3(double ttime);

	void buildAttackCurves();
	void placeAttackNode(int sk);
	void disableAttack();
	void scaleCurve();
	
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
		curves.push_back(vector<Vector3d>(chainSize));
		attackCurves.push_back(vector<Vector3d>(chainSize));
		positioningStrengths = rigidnessStrengths = vector<double> (chainSize, 0);
		lastMinIndexes = vector<int> (chainSize, 0);

		for (int i = 0; i < chainSize; ++i) {
			currentPositions[nextSK][i] = lastPositions[nextSK][i] = restPositions[nextSK][i] = c->positions[i];
			lastMinIndexes[i] = i;
		}
	}

	void setPositions(skeleton* s) {

		chainSize = index2 - index1 + 1;

		restPositions.resize(inputs.size());
		lastPositions.resize(inputs.size());
		currentPositions.resize(inputs.size());
		idealPositions.resize(inputs.size());
		curves.resize(inputs.size());
		attackCurves.resize(inputs.size());
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

