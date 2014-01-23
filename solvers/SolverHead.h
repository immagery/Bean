#pragma once
#include "Solver.h"

class SolverHead : public Solver {
public:
	// SolverHead works after SolverLook and moves the head towards the lookPoint
	Vector3d lookPoint;
	double alpha;		// 0 = head, 1 = lookPoint

	bool moving;
	double radius;
	int f;
	int maxF;

	// Constructors and destructors
	SolverHead(void);
	~SolverHead(void) {}

	void solve(SolverData* data);
	void updateDirtyness();
	virtual void solve();

	
};

