#include "Intelligence.h"
#include <utils\util.h>

using namespace std;
using namespace Eigen;

#define G_THITA M_PI/6
#define G_PHI 0
Intelligence::Intelligence(int sk) 
{
	currentState = IDLE;
	thita = (double)rand() / RAND_MAX * 2 * M_PI;
	phi = (double)rand() / RAND_MAX * M_PI;
	id = sk;
	ampMultiplier = 1;
	freqMultiplier = 1;

	// Look-at variables.
	lookInit = false;
	lookPointRadius = rand()%20 + 20;
	lookPoint = Vector3d(0,0,0);
	globalLookPoint = Vector3d(0,0,0);
	globalLookPointRadius = 700;
	globalThita = fRand(0, M_PI/6.0);
	globalPhi = fRand(-M_PI/6, M_PI/6);
	globalPhi = G_PHI;
	globalThita = G_THITA;

	// Head position solver
	headMovDirection = Vector3d(0,0,0);

	headPosition = NULL;
	look = NULL;
}

void Intelligence::setState(States st) {
	if(st >= IDLE && st <= NONE)
	currentState = st;
}

void Intelligence::think() 
{
	//printf("Thinking!\n");

	// towards the target
	updateHeadPosition();

	// Look-at
	updateLookPoint();

	// Ocilation depending on the state.
	updateOscillationParameters();
}

void Intelligence::updateLookPoint() {
	// Select a random area using global sphere and random values for thita and phi
	// Random val = min + random between 0..(max - min)

	if(!look) return;

	Vector3d p = globalLookPoint;
	p.x() += globalLookPointRadius * sin(globalThita) * sin(globalPhi);
	p.y() += globalLookPointRadius * cos(globalThita);
	p.z() += globalLookPointRadius * sin(globalThita) * cos(globalPhi);

	thita += 0.01;
	phi += 0.03;

	if (currentState == IDLE) 
		look->lookPoint = lookPoint;
	else if (currentState == LOOKAT)				
		look->lookPoint = p;

	look->lookPoint.x() += lookPointRadius * sin(thita) * sin(phi);
	look->lookPoint.y() += lookPointRadius * cos(thita);
	look->lookPoint.z() += lookPointRadius * sin(thita) * cos(phi);
}

void Intelligence::updateOscillationParameters() 
{
	if(!sinus) return;

	sinus->multAmp = ampMultiplier;
	sinus->multFreq = freqMultiplier;
}

void Intelligence::updateHeadPosition()
{
	// update position of the head

	// Between Lmax y Lmin changing with a energy function.

	// 1. if it is a border zone, the direction needs to change faster in the opposite direction.
	
	// There will be some interesting things in the field, and some attitudes.
	// 1. If something interests, the head will be going to there, when reaches the max value: goes back a little and to some 
	// direction and tries again.
	
	if(!headPosition) return;

	printf("[Intelligence.cpp] Update head position \n");

	headMovDirection = Vector3d(fRand(-1, 1), fRand(-1, 1), fRand(-1, 1));

	if(headPosition->desiredPos.y() > headPosition->Lmax_sub)
		headMovDirection = Vector3d(fRand(-1, 1), fRand(0, -1), fRand(-1, 1));

	headPosition->desiredPos = headPosition->desiredPos + headMovDirection;

}
