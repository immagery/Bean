#ifndef SNAKE_BRAIN_H
#define SNAKE_BRAIN_H

#include <Eigen/Core>
//#include "solvers/AllSolvers"

#include <DataStructures/axis.h>
#include <snakeVertebra.h>

using namespace std;
using namespace Eigen;

#define G_THITA M_PI/6
#define G_PHI 0

enum snake_state {IDLE = 0, LOOKAT, EXCITED, ANGRY, NONE};

class snakeBrain {
	public:

	snakeBrain();
	~snakeBrain(){}

	void setState(snake_state st);
	void think(float timeStep);

	void initOscilationMovement(float _ampMultiplier, float _freqMultiplier);

	void setHeadVertebra(snakeVertebra* head);
	void setBaseVertebra(snakeVertebra* base);
	
	void initTime();
	void moveToPosition(Vector3d pos);

	void drawFunc();

	// Define all states
	snake_state currentState;

	// Control to the snake
	snakeVertebra* headVertebra;
	snakeVertebra* baseVertebra;

	float snakeLength;

	// Reference axis
	axis baseAxis;
	axis headAxis;
	axis realWorldAxis;

	float timer;

	Vector3d lookPoint;
	Vector3d whereToGo;

	double thita, phi;		// thita = -pi/2..pi/2 (altitude), phi = 0..2pi
	double ampMultiplier;
	double freqMultiplier;

	float pointDistanceX;
	float pointDistanceY;

	Vector3d objective;

	/*
	// Looking stuff
	Vector3d lookPoint, globalLookPoint;
	double lookPointRadius, globalLookPointRadius;
	
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
	axis realWorldAxis;
	*/

};

#endif // SNAKE_BRAIN_H