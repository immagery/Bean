#pragma once
#include "Solver.h"

class SolverHead : public Solver {
public:
	// SolverHead works after SolverLook and moves the head towards the lookPoint
	Vector3d desiredPos;		// point where the snake wants to move its head to 

	Vector3d desplFromDesiredPos;

	double alpha;				// speed parameter
	bool moving;				// flag to indicate if the snake wants to move or not
	double radius;				// maximum allowed radius

	float Lmax;
	float Lmax_sub;
	float Xmax;
	Vector3d dir;

	float seed;

	// Mini verlet
	double lastTime;
	Vector3d lastPosition;


	// Constructors and destructors
	SolverHead(void);
	~SolverHead(void) {}

	void solve(SolverData* data);
	void updateDirtyness();
	virtual void solve();

	
};

