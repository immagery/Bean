#pragma once

#include "Solver.h"
#include "skeleton.h"
#include <Eigen/Core>

class SolverDir : public Solver {
public:
	SolverDir(int _id) : Solver(_id) 
	{
		qrot = Quaterniond::Identity();
	}
	~SolverDir(void) {}

	Quaterniond qrot;

	virtual void solve(SolverData* data) {}

	virtual void updateDirtyness() {
		if (!qrot.isApprox(data->dirRot, 0.1)) {
			dirtyFlag = true;
			propagateDirtyness();
		}
	}

	virtual void solve () {
		qrot = data->dirRot;
		for (int i = 0; i < inputs.size(); ++i) {
			Chain* ichain = inputs[i];
			Chain* ochain = outputs[i];
			ochain->positions[0] = ichain->positions[0];
			for (int j = 1; j < ochain->positions.size(); ++j) {
				Vector3d increment = ichain->positions[j] - ichain->positions[j-1];
				increment = qrot._transformVector(increment);
				ochain->positions[j] = ochain->positions[j-1] + increment;
			}
		}
	}
};