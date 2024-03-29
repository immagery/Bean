#pragma once
#include "Solver.h"

class SolverLook : public Solver {
public:

	SolverLook(int _id) : Solver(_id)
	{ 
		index1 = index2 = -1; 
		qrot = Quaterniond::Identity();
		restLookVector = lookPoint = Vector3d(0,0,0);
	}
	~SolverLook(void) {}

	int index1, index2;
	Quaterniond qrot;
	Vector3d restLookVector, lookPoint;
	double restDistance;

	void solve(SolverData* data) {
		lookPoint = data->lookPoint;
		solve();
	}

	void solve () {
		clock_t start = clock();
		for (int i = 0; i < inputs.size(); ++i) {
			Chain* ichain = inputs[i];
			for (int j = 0; j < ichain->positions.size(); ++j)
				outputs[i]->positions[j] = ichain->positions[j];

			Vector3d firstNode = inputs[i]->positions[index1];
			Vector3d secondNode = inputs[i]->positions[index2];

			//Vector3d lookVector = qrot._transformVector(restLookVector);
			Vector3d lookVector = qrot._transformVector(secondNode - firstNode);
			Vector3d desiredVector = lookPoint - firstNode;

			Quaterniond r;	r.setFromTwoVectors(lookVector, desiredVector);
			Vector3d rotatedVector = r._transformVector(secondNode - firstNode);
			Vector3d pointToLook = firstNode + rotatedVector.normalized()*restDistance;

			outputs[i]->positions[index2] = pointToLook;
		}
		clock_t end = clock();
		//printf("	Elapsed look time: %f\n", timelapse(start,end));
	}
};

