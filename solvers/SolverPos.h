#pragma once

#include "Solver.h"
#include "skeleton.h"
#include <Eigen/Core>

class SolverPos : public Solver {
public:

	// Attributes
	Vector3d baseTranslation;
	Quaterniond baseRotation;
	Vector3d restTrans;

	SolverPos(int _id) : Solver(_id) 
	{ 
		baseTranslation = Vector3d(0,0,0);
		baseRotation = Quaterniond::Identity();
	}
	~SolverPos(void);

	virtual void updateDirtyness() {
		dirtyFlag = true;
		propagateDirtyness();
	}

	virtual void solve(SolverData* data);
	virtual void solve ();
};