#include "SolverHead.h"
#include "AdriViewer.h"

SolverHead::SolverHead()  : Solver() { 
	alpha = 1;	
	moving = true; 
	lastPosition = Vector3d(0,0,0);
	lastTime = 0;
}


void SolverHead::solve(SolverData* data) {
	time = data->time;
	desiredPos = data->lookPoint;
	alpha = data->alpha;
	solve();
}

void SolverHead::updateDirtyness() {
	dirtyFlag = true;
	//dirtyFlag = (alpha != data->alpha || lookPoint != data->lookPoint);
	propagateDirtyness();
}

void SolverHead::solve() {
	desiredPos = data->desiredPos;
	alpha = data->alpha;

	for (int ip = 0; ip < inputs.size(); ++ip) {

		for (int j = 0; j < outputs[ip]->positions.size(); ++j) {
			outputs[ip]->positions[j] = inputs[ip]->positions[j];
		}

		if (moving) {
			//outputs[ip]->positions[index1] = desiredPos;
			//continue;

			desiredPos = inputs[ip]->positions[0] + desiredPos;
			desiredPos.x() += 40*sin(data->time);
			desiredPos.x() += 10*cos(data->time);

			glPointSize(15);
			glDisable(GL_LIGHTING);
			glBegin(GL_POINTS);
			glVertex3d(desiredPos.x(), desiredPos.y(), desiredPos.z());
			glEnd();
			glEnable(GL_LIGHTING);

			// MiniVerlet simulation
			double deltaTime = data->time - lastTime;
			lastTime = data->time;
			Vector3d p = inputs[ip]->positions[index1];
			Vector3d currentDist = desiredPos - p;
			Vector3d delta1 = currentDist / currentDist.norm() * 100 * currentDist.norm();
			Vector3d vel1 = (p - lastPosition);
			double v1 = (vel1).dot(currentDist.normalized());
			Vector3d damp1 = currentDist / currentDist.norm() * 10 * v1;
			Vector3d force1 = (delta1+damp1);
			p += force1 * deltaTime * alpha;
			if (alpha == 0) p = lastPosition;

			Vector3d velocity = (p - lastPosition)*0.9;
			//velocity = Vector3d(0,0,0);
			Vector3d nextPos = p + velocity;
			lastPosition = p;
			outputs[ip]->positions[index1] = nextPos;

			// Update point

			//Vector3d direction = desiredPos - inputs[ip]->positions[index1];
			//Vector3d nextPos = inputs[ip]->positions[index1] + 1*direction;
			//outputs[ip]->positions[index1] = nextPos;
		}



	}
}