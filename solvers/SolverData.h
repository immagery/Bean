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
	Vector3d lookPoint;
	Vector3d baseTranslation;
	Quaterniond baseRotation;
	Quaterniond dirRot;
	Quaterniond neck;

	// Look
	Vector3d lookPointRadius;
	double thita, phi;

	// Oscillation
	double ampMultiplier, freqMultiplier;

	bool rigidness;

	// Head movement
	double alpha;
	Vector3d headOffset;

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
		alpha = 0;
		headOffset = Vector3d(0,0,0);
	}
	~SolverData(void) {}
};

