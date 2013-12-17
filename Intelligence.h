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
	double lookPointRadius, globalLookPointRadius;
	double thita, phi;		// thita = -pi/2..pi/2 (altitude), phi = 0..2pi
	double globalThita, globalPhi;
	SolverLook* look;
	Vector3d restUpVector;	bool lookInit;

	// Oscillating stuff
	double ampMultiplier;
	double freqMultiplier;
	SolverSinusoidal* sinus;

	Intelligence(int sk) {
		currentState = IDLE;
		thita = (double)rand() / RAND_MAX * 2 * M_PI;
		phi = (double)rand() / RAND_MAX * M_PI;
		lookPointRadius = rand()%20 + 20;
		lookPoint = Vector3d(0,0,0);
		globalLookPoint = Vector3d(0,480, 400);
		id = sk;
		ampMultiplier = 1;
		freqMultiplier = 1;

		lookInit = false;

		globalLookPoint = Vector3d(0,0,0);
		globalLookPointRadius = 700;
		globalThita = fRand(0, M_PI/6.0);
		globalPhi = fRand(-M_PI/6, M_PI/6);
		globalPhi = 0;
		globalThita = M_PI/12;
	}

	void setState(int i) {
		if (i == 0) currentState = IDLE;
		if (i == 1) currentState = LOOKAT;
	}

	void think() {
		updateLookPoint();
		updateOscillationParameters();
	}

	double fRand(double fMin, double fMax) {
		double f = (double)rand() / RAND_MAX;
		return fMin + f * (fMax - fMin);
	}

	void updateLookPoint() {
		// Select a random area using global sphere and random values for thita and phi
		// Random val = min + random between 0..(max - min)

		Vector3d p = globalLookPoint;
		p.x() += globalLookPointRadius * sin(globalThita) * sin(globalPhi);
		p.y() += globalLookPointRadius * cos(globalThita);
		p.z() += globalLookPointRadius * sin(globalThita) * cos(globalPhi);

		thita += 0.01;
		phi += 0.03;
		if (currentState == IDLE) 
			look->lookPoint = lookPoint;
			//look->lookPoint = p;
		else						
			look->lookPoint = globalLookPoint;

		look->lookPoint.x() += lookPointRadius * sin(thita) * sin(phi);
		look->lookPoint.y() += lookPointRadius * cos(thita);
		look->lookPoint.z() += lookPointRadius * sin(thita) * cos(phi);
	}

	void updateOscillationParameters() {
		sinus->multAmp = ampMultiplier;
		sinus->multFreq = freqMultiplier;
	}
};

#endif