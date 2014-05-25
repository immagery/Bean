#include "snakeBrain.h"
#include <utils\util.h>

#include <utils/utilGL.h>

using namespace std;
using namespace Eigen;

#define MARGIN_DISTANCE 0.01

#define G_THITA M_PI/6
#define G_PHI 0
snakeBrain::snakeBrain() 
{
	currentState = IDLE;

	thita = (double)rand() / RAND_MAX * 2 * M_PI;
	phi = (double)rand() / RAND_MAX * M_PI;
	
	ampMultiplier = 8;
	freqMultiplier = 5;

	pointDistanceX = 1;
	pointDistanceY = 1;

	snakeLength = 1;

	baseVertebra = NULL;
	headVertebra = NULL;
	// Look-at variables.
	/*
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
	*/
}

void snakeBrain::initOscilationMovement(float _ampMultiplier, float _freqMultiplier)
{
	ampMultiplier =  _ampMultiplier;
	freqMultiplier =  _freqMultiplier;
}

void snakeBrain::setState(snake_state st) 
{
	if(st >= IDLE && st <= NONE)
	currentState = st;
}

void snakeBrain::moveToPosition(Vector3d pos)
{
	headAxis.pos = pos;
}



void snakeBrain::think(float timeStep) 
{
	if(!baseVertebra || !headVertebra ) return;

	//printf("Thinking!\n");

	timer += timeStep;

	while((objective-baseVertebra->center->position).norm() > snakeLength/2 || fabs((objective - headAxis.pos).norm()) < 1)
	{
		float lengthAdjusted = snakeLength/2;
		float lengthMin = lengthAdjusted - MARGIN_DISTANCE;
		float x = ((double)rand())/RAND_MAX*lengthMin-lengthMin;
		float y = sqrt(lengthMin*lengthMin-x*x);

		objective = Vector3d(y, 0, x) + baseVertebra->center->position;

		printf("Nuevo Objetivo\n");
	}

	Vector3d oscilationMovement;
	Vector3d positionMovement;

	//Animar la cabeza como si fuera una serpiente... movimiento sinusoidal achatado con cierto retraso
	//float Amplitude = 8; 
	//float Frequency = 5;
	Vector3d reposition;
	if(currentState == IDLE)
	{
		float zOscilation = sin(timer/freqMultiplier)*ampMultiplier;
		float frontMovement = timeStep/5.0;  

		oscilationMovement = Vector3d(0,0,zOscilation);
		//positionMovement = Vector3d(frontMovement, 0, 0);
		reposition = objective-headAxis.pos;
		
		positionMovement = reposition.normalized()*frontMovement*10;

	}
	else if(currentState = LOOKAT)
	{
		float zOscilation = sin(timer/freqMultiplier)*ampMultiplier;
		float frontMovement = timeStep/2.0;

		oscilationMovement = Vector3d(0,0,zOscilation);
		positionMovement = Vector3d(frontMovement, 0, 0);
	}
	else if(currentState =  EXCITED)
	{
		float zOscilation = sin(timer/(freqMultiplier/5.0))*ampMultiplier;
		float frontMovement = 0;

		oscilationMovement = Vector3d(0,0,zOscilation);
		positionMovement = Vector3d(frontMovement, 0, 0);
	}
	else if(currentState = ANGRY)
	{
		float zOscilation = sin(timer/(freqMultiplier*5.0))*ampMultiplier;
		float frontMovement = 0;

		oscilationMovement = Vector3d(0,0,zOscilation);
		positionMovement = Vector3d(frontMovement, 0, 0);
	}

	oscilationMovement = headAxis.rot._transformVector(oscilationMovement);
	//positionMovement = headAxis.rot._transformVector(positionMovement);

	Vector3d anteriorPos = headAxis.pos;

	headAxis.pos += positionMovement;
	Vector3d headPosition = headAxis.pos + oscilationMovement;

	if(((headPosition-baseVertebra->center->position).norm() - snakeLength ) > - MARGIN_DISTANCE)
	{
		printf("Hemos llegado al tope, deberiamos rectificar\n");

		//headPosition = (headPosition-baseVertebra->center->position).normalized()* (snakeLength-MARGIN_DISTANCE);
		Quaterniond d = Quaterniond::Identity();
		d.setFromTwoVectors(Vector3d(1,0,0), Vector3d(1,0,0) - oscilationMovement.normalized()*0.01);
		headAxis.rot *= d;
	}

	if(headVertebra)
	{
		headVertebra->center->position = headPosition;

		Vector3d xDir(pointDistanceX, 0, 0);
		Vector3d yDir(0, pointDistanceY, 0);

		Quaterniond q2 = Quaterniond::Identity();
		q2.setFromTwoVectors(Vector3d(1,0,0), reposition.normalized());

		Quaterniond q3 = Quaterniond::Identity();
		q3 = q3.slerp(0.1,q2);

		//headAxis.rot = q2;
		
		/*
		xDir = headAxis.rot._transformVector(xDir);

		Quaterniond q2 = Quaterniond::Identity();
		q2.setFromTwoVectors(xDir.normalized(), reposition.normalized());

		Quaterniond q3 = Quaterniond::Identity();
		q3 = q3.slerp(0.1,q2);

		xDir = q3._transformVector(xDir);
		*/
		xDir = (headAxis.rot)._transformVector(xDir);
		yDir = (headAxis.rot)._transformVector(yDir);

		headVertebra->x->position =  xDir + headPosition;
		headVertebra->y->position =  yDir + headPosition;
	}
	

	// towards the target
	//updateHeadPosition();

	// Look-at
	//updateLookPoint();

	// Ocilation depending on the state.
	//updateOscillationParameters();
}

void snakeBrain::setBaseVertebra(snakeVertebra* base)
{
	// Get the pointer
	baseVertebra = base;

	// Get initial data for computations
	baseAxis.pos = base->center->position;

	Vector3d y = Vector3d(0,1,0);
	Vector3d x = Vector3d(1,0,0);

	Vector3d xRef = base->x->position - base->center->position;
	Vector3d yRef = base->y->position - base->center->position;

	Quaterniond q1 = Quaterniond::Identity();
	q1.setFromTwoVectors(x, xRef);
			
	y = q1._transformVector(y);
	Quaterniond q2 = Quaterniond::Identity();
	q2.setFromTwoVectors(y, yRef);

	headAxis.rot = q1*q2;

	// Init vertebra constraints
	base->center->weight = 0;
	base->x->weight = 0;
	base->y->weight = 0;

	pointDistanceX = (base->center->position-base->x->position).norm();
	pointDistanceY = (base->center->position-base->y->position).norm();

	if(headVertebra)
	{
		snakeLength = (baseVertebra->center->position - headVertebra->center->position).norm();
	}

}

void snakeBrain::setHeadVertebra(snakeVertebra* head)
{
	// Get the pointer
	headVertebra = head;

	// Get initial data for computations
	headAxis.pos = head->center->position;

	Vector3d y = Vector3d(0,1,0);
	Vector3d x = Vector3d(1,0,0);

	Vector3d xRef = head->x->position - head->center->position;
	Vector3d yRef = head->y->position - head->center->position;

	Quaterniond q1 = Quaterniond::Identity();
	q1.setFromTwoVectors(x, xRef);
			
	y = q1._transformVector(y);
	Quaterniond q2 = Quaterniond::Identity();
	q2.setFromTwoVectors(y, yRef);

	headAxis.rot = q1*q2;

	objective = headAxis.pos;

	// Init vertebra constraints
	head->setAsHead();

	pointDistanceX = (head->center->position-head->x->position).norm();
	pointDistanceY = (head->center->position-head->y->position).norm();

	if(baseVertebra)
	{
		snakeLength = (baseVertebra->center->position - headVertebra->center->position).norm();
	}

	// Animation parameters, TOMOVE
	//part0 = head->center->position;
	//part1 = head->x->position;
	//part2 = head->y->position;
	// END - animation parameters
}

void snakeBrain::initTime()
{
	timer = 0; 
}

void snakeBrain::drawFunc()
{
	glColor3f(1, 0, 0);

	drawPointLocator(objective, 1, false);
}

/*
void snakeBrain::updateLookPoint() {
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

void snakeBrain::updateOscillationParameters() 
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
*/