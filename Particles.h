#pragma once
#include "DataStructures/Object.h"
#include "DataStructures/SurfaceData.h"

class Particles : public object {
public:
	SurfaceGraph *graph;
	vector<vector<Point3d> > positions;		// baked simulation

	vector<Point3d> restPositions;
	vector<Point3d> currentPositions;
	vector<Point3d> lastPositions;

	double lastTime;

	double stiffness;
	double g;

	double xvalue;

	Particles(void);
	~Particles(void);
	void drawFunc(int frame);
	void drawFunc();
	void bake(int maxFrames, double deltaTimePerFrame);
	void solve(double time);
};

