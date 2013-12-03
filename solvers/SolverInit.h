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

	virtual void solve () {
		baseRotation = data->baseRotation;
		baseTranslation = data->baseTranslation;
		for (int i = 0; i < outputs.size(); ++i) {
			Chain* chain = outputs[i];
			Quaterniond rotation = baseRotation;
			chain->positions[0] = initialTranslation + baseTranslation;
			for (int j = 0; j < chain->positions.size(); ++j) {
				//if (j == 0) translation = baseTranslation + rotation._transformVector(initPositions[j]);
				//else		translation = chain->positions[j-1] + rotation._transformVector(initPositions[j]);
				if (j != 0) chain->positions[j] = chain->positions[j-1] + rotation._transformVector(initPositions[j]);
				Quaterniond ori = initQORIENTS[j];
				Quaterniond rot = initQROTS[j];
				Quaterniond qrot = ori * rot;
				rotation = (rotation * qrot).normalized();
			}
		}
	}
};