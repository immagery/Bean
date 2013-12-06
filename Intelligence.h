#ifndef INTELLIGENCE_H
#define INTELLIGENCE_H

#include <Eigen/Core>
#include "solvers/AllSolvers"

using namespace std;
using namespace Eigen;

class Intelligence {
	public:

	// Define all states
	enum States {IDLE, LOOKAT, EXCITED, ANGRY};
	States currentState;

	int id;	// useful to give different behaviours

	// Looking stuff
	Vector3d lookPoint, globalLookPoint;
	double lookPointRadius;
	double thita, phi;		// thita = 0..2pi, phi = -pi.. pi
	SolverLook* look;

	Intelligence(int sk) {
		currentState = IDLE;
		thita = (double)rand() / RAND_MAX * 2 * M_PI;
		phi = (double)rand() / RAND_MAX * M_PI;
		lookPointRadius = rand()%20 + 20;
		lookPoint = Vector3d(0,480,400);
		globalLookPoint = Vector3d(0,480, 400);
		id = sk;
	}

	void setState(int i) {
		if (i == 0) currentState = IDLE;
		if (i == 1) currentState = LOOKAT;
	}

	void think() {
		updateLookPoint();
	}

	void updateLookPoint() {
		thita += 0.01;
		phi += 0.03;
		if (currentState == IDLE) 
			look->lookPoint = lookPoint;
		else						
			look->lookPoint = globalLookPoint;

		look->lookPoint.x() += lookPointRadius * sin(thita) * cos(phi);
		look->lookPoint.y() += lookPointRadius * sin(thita) * sin(phi);
		look->lookPoint.z() += lookPointRadius * cos(thita);
	}
};

#endif