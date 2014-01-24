#pragma once

#include "Solver.h"
#include "skeleton.h"
#include <Eigen/Core>

class SolverInit : public Solver {
public:
	SolverInit(void) : Solver() { 
		initQROTS = vector<Quaterniond>();
		initQORIENTS = vector<Quaterniond>();
		initPositions = vector<Vector3d>();
		baseRotation = Quaterniond::Identity();
		baseTranslation = Vector3d(0,0,0);
	}
	~SolverInit(void);

	joint *jt;

	vector<Quaterniond> initQROTS;
	vector<Quaterniond> initQORIENTS;
	vector<Vector3d> initPositions;
	
	Quaterniond baseRotation;
	Vector3d baseTranslation;
	Vector3d initialTranslation;

	virtual void updateDirtyness() {
		dirtyFlag = true;
		propagateDirtyness();
		/*if (!baseRotation.isApprox(data->baseRotation, 0.0001) || !data->baseTranslation.isApprox(baseTranslation, 0.0001)) {
			dirtyFlag = true;
			propagateDirtyness();
		}*/
	}

	virtual void solve(SolverData* data) {
	}

	void setPositions(skeleton* s) {
		chainSize = (index2 - index1 + 1) + 1;
		inputs = vector<Chain*> (1);
		inputs[0] = new Chain();	inputs[0]->positions = vector<Vector3d>(chainSize);
		for (int i = index1; i <= index2; ++i)
			initPositions.push_back(s->joints[i]->translation);
		Vector3d lastPos(0,0,0);	int n = 0;
		for (int i = index2+1; i < s->joints.size(); ++i) {
			lastPos = lastPos +  s->joints[i]->translation;
			++n;
		}
		lastPos = lastPos / n;
		initPositions.push_back(lastPos);
	}

	Chain* initialChain() {
		Chain* chain = new Chain;	chain->positions.resize(chainSize);
		chain->positions[0] = Vector3d(0,0,0);
		for (int j = 0; j < chain->positions.size(); ++j)
			chain->positions[j] = initPositions[j];
		return chain;
	}

	void initialize() {
		Chain* rest = initialChain();
		for (int ip = 0; ip < inputs.size(); ++ip) {
			for (int j = 0; j < inputs[ip]->positions.size(); ++j) {
				inputs[ip]->positions[j] = rest->positions[j];
			}

			// Apply slight sin and lower the head to build an initial curve
			for (int j = 1; j < inputs[ip]->positions.size(); ++j) {
				inputs[ip]->positions[j].x() += (10+rand()%25)*sin(j);
			}
		}
	}

	virtual void solve () {
		for (int ip = 0; ip < inputs.size(); ++ip) {
			for (int j = 0; j < inputs[ip]->positions.size(); ++j)
				outputs[ip]->positions[j] = inputs[ip]->positions[j];
		}
	}
};