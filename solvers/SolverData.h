#pragma once

#include "Chain.h"
#include "skeleton.h"
#include <Eigen/Core>
#include <Eigen\Geometry>

using namespace Eigen;

// Contains all necessary info for the solvers to work
class SolverData {
public:
	// Time data
	double time, fps, gravity;

	// References of position
	Vector3d lookPoint;
	Vector3d baseTranslation;
	Quaterniond baseRotation;
	Quaterniond dirRot;
	Quaterniond neck;

	// Head & look
	double alpha, radius;
	Vector3d desiredPos;
	Vector3d lookPointRadius;
	double thita, phi;

	// Oscillation
	double ampMultiplier, freqMultiplier;

	bool rigidness;

	// Skeletons
	vector<skeleton*> skeletons;

	SolverData(void) {
		time = fps = gravity = 0;
		lookPoint = Vector3d(0,0,0);
		baseRotation = Quaterniond::Identity();
		dirRot = Quaterniond::Identity();
		baseTranslation = Vector3d(0,0,0);
		ampMultiplier = freqMultiplier = 1;
		rigidness = true;
		skeletons = vector<skeleton*>(0);
		alpha = 0.1;
		radius = 300;
		desiredPos = Vector3d(0,480,0);
	}
	~SolverData(void) {}
};

