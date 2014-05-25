#ifndef INTELLIGENCE_H
#define INTELLIGENCE_H

#include <Eigen/Core>
#include "solvers/AllSolvers"

#include <DataStructures/axis.h>

using namespace std;
using namespace Eigen;

#define G_THITA M_PI/6
#define G_PHI 0

class Intelligence {
	public:

	Intelligence(int sk);
	~Intelligence(){}

	// Define all states
	enum States {IDLE = 0, LOOKAT, EXCITED, ANGRY, NONE};
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

	//Position Stuff
	SolverHead* headPosition;
	Vector3d headMovDirection;
	
	void setState(States st);
	void think();
	void updateLookPoint();
	void updateOscillationParameters();
	void updateHeadPosition();


	//reference axis to of the solver.
	axis baseAxis;
	axis headAxis;
	axis realWorldAxis;

};

#endif