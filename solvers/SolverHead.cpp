#include "SolverHead.h"

SolverHead::SolverHead()  : Solver() { alpha = 0;	f = maxF = 0;	moving = false;		radius = 450;	lookPoint = Vector3d(0,0,0); }


void SolverHead::solve(SolverData* data) {
	time = data->time;
	lookPoint = data->lookPoint;
	alpha = data->alpha;
	solve();
}

void SolverHead::updateDirtyness() {
	dirtyFlag = true;
	dirtyFlag = (alpha != data->alpha || lookPoint != data->lookPoint);
	propagateDirtyness();
}

void SolverHead::solve() {
	lookPoint = data->lookPoint;
	alpha = data->alpha;

	for (int i = 0; i < inputs.size(); ++i) {
		Chain* ichain = inputs[i];
		for (int j = 0; j < ichain->positions.size(); ++j)
			outputs[i]->positions[j] = ichain->positions[j];

		Vector3d secondNode = outputs[i]->positions[index2];

		Vector3d lookDirection = (lookPoint - secondNode).normalized();

		if (moving) {
			++f;
			 if ((outputs[i]->positions[index2] + f*lookDirection*0.8 - outputs[i]->positions[0]).norm() < radius)  maxF = f;
			 else lookDirection = Vector3d(-1,-0.2,0);
		}
		else if (!moving) f = max(f-1,0);

		outputs[i]->positions[index2] += maxF*lookDirection*0.8;
		if ((outputs[i]->positions[index2] + (maxF+1)*lookDirection*0.8 - outputs[i]->positions[0]).norm() < radius)
			outputs[i]->positions[index2].x() += sin(f/100.0)*50;
		else 
			outputs[i]->positions[index2].z() += sin(f/100.0)*50;
		//Vector3d pointToMove = secondNode + alpha * lookDirection;

		//outputs[i]->positions[index2] = pointToMove;
		outputs[i]->positions[index2] += data->headOffset;
	}
}