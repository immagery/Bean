#pragma once
#include "Solver.h"

class SolverLook : public Solver {
public:

	SolverLook(void) { 
		index1 = index2 = -1; 
		qrot = Quaterniond::Identity();
		restLookVector = lookPoint = Vector3d(0,0,0);
	}
	~SolverLook(void) {}

	int index1, index2;
	Quaterniond qrot;
	Vector3d restLookVector, lookPoint;

	void solve(SolverData* data) {
		lookPoint = data->lookPoint;
		solve();
	}

	void solve () {
		for (int i = 0; i < inputs.size(); ++i) {
			Chain* ichain = inputs[i];
			for (int j = 0; j < ichain->positions.size(); ++j)
				outputs[i]->positions[j] = ichain->positions[j];

			Vector3d firstNode = outputs[i]->positions[index1];
			Vector3d secondNode = outputs[i]->positions[index2];

			//Vector3d lookVector = qrot._transformVector(restLookVector);
			Vector3d lookVector = qrot._transformVector(secondNode - firstNode);
			Vector3d desiredVector = lookPoint - firstNode;

			Quaterniond r;	r.setFromTwoVectors(lookVector, desiredVector);
			Vector3d rotatedVector = r._transformVector(secondNode - firstNode);
			Vector3d pointToLook = firstNode + rotatedVector;

			outputs[i]->positions[index2] = pointToLook;
		}
	}
};

