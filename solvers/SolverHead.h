#pragma once
#include "Solver.h"

class SolverHead : public Solver {
public:
	// SolverHead works after SolverLook and moves the head towards the lookPoint
	Vector3d lookPoint;
	double alpha;		// 0 = head, 1 = lookPoint

	// Constructors and destructors
	SolverHead(void) : Solver() { alpha = 0;	lookPoint = Vector3d(0,0,0); }
	~SolverHead(void) {}

	void solve(SolverData* data);
	void updateDirtyness();
	virtual void solve();

	
};

