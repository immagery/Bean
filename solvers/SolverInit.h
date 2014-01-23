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
		if (!baseRotation.isApprox(data->baseRotation, 0.0001) || !data->baseTranslation.isApprox(baseTranslation, 0.0001)) {
			dirtyFlag = true;
			propagateDirtyness();
		}
	}

	virtual void solve(SolverData* data) {
	}

	void setPositions(skeleton* s) {
		chainSize = (index2 - index1 + 1) + 1;		// adding one more one to account for the last "median" node
		for (int i = index1; i <= index2; ++i) {
			initPositions.push_back(s->joints[i]->pos);
			initQORIENTS.push_back(s->joints[i]->qOrient);
			initQROTS.push_back(s->joints[i]->qrot);
		}
		// Create last node
		Vector3d lastPos(0,0,0);	int n = 0;
		for (int i = index2+1; i < s->joints.size(); ++i) {
			lastPos = lastPos +  s->joints[i]->translation;
			++n;
		}
		lastPos = lastPos / n;
		initPositions.push_back(lastPos);
		initQORIENTS.push_back(Quaterniond::Identity());
		initQROTS.push_back(Quaterniond::Identity());
	}

	Chain* initialChain() {
		Chain* chain = new Chain;	chain->positions.resize(chainSize);
		Quaterniond rotation = baseRotation;
		chain->positions[0] = initialTranslation + baseTranslation;
		for (int j = 0; j < chain->positions.size()-1; ++j) {
			//if (j == 0) translation = baseTranslation + rotation._transformVector(initPositions[j]);
			//else		translation = chain->positions[j-1] + rotation._transformVector(initPositions[j]);
			if (j != 0) chain->positions[j] = chain->positions[j-1] + rotation._transformVector(initPositions[j]);
			Quaterniond ori = initQORIENTS[j];
			Quaterniond rot = initQROTS[j];
			Quaterniond qrot = ori * rot;
			rotation = (rotation * qrot).normalized();
		}
		chain->positions[chain->positions.size()-1] = baseTranslation + initPositions[chain->positions.size()-1];
		return chain;
	}

	virtual void solve () {
		clock_t start = clock();
		baseRotation = data->baseRotation;
		baseTranslation = data->baseTranslation;
		for (int i = 0; i < outputs.size(); ++i) {
			Chain* chain = outputs[i];
			Quaterniond rotation = baseRotation;
			chain->positions[0] = initialTranslation + baseTranslation;
			for (int j = 0; j < chain->positions.size()-1; ++j) {
				//if (j == 0) translation = baseTranslation + rotation._transformVector(initPositions[j]);
				//else		translation = chain->positions[j-1] + rotation._transformVector(initPositions[j]);
				if (j != 0) chain->positions[j] = chain->positions[j-1] + rotation._transformVector(initPositions[j]);
				Quaterniond ori = initQORIENTS[j];
				Quaterniond rot = initQROTS[j];
				Quaterniond qrot = ori * rot;
				rotation = (rotation * qrot).normalized();
			}
			chain->positions[chain->positions.size()-1] = baseTranslation + initPositions[chain->positions.size()-1];
		}
		clock_t end = clock();
		//printf("	Elapsed init time: %f\n", timelapse(start,end));
	}
};