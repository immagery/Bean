#include "SolverHead.h"
#include "AdriViewer.h"

SolverHead::SolverHead()  : Solver() { 
	alpha = 1;	
	moving = true; 
	lastPosition = Vector3d(0,0,0);
	lastTime = 0;

	Lmax = 350;
	Lmax_sub = Lmax - 30;
	dir = Vector3d(0,1,0);

	seed = fRand(3.0, 10.0);
	desplFromDesiredPos = Vector3d(0,0,0);
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
			//desiredPos.x() += 5*sin(data->time)*fRand(-1,1);
			//desiredPos.y() += 5*cos(data->time)*fRand(-1,1);
			printf("time: %f\n",data->time);
			dir.y() = (sin(data->time-50)*sin(data->time*seed*0.1)*seed*seed);//(sin((data->time)/50*(2*M_PI)*100*seed)-0.5)*0,1;
			dir.x() = (cos(data->time-10)*cos(data->time*seed*0.1)*seed*seed);//(cos((data->time)/50*2*M_PI*seed*200)-0,5)*0,1;

			Vector3d newPoint = desiredPos;
			desplFromDesiredPos = dir;

			if((desiredPos+desplFromDesiredPos).y() > Lmax && desplFromDesiredPos.y() > 0) 
				desplFromDesiredPos.y() = desplFromDesiredPos.y() * (1-((desiredPos+desplFromDesiredPos).y()-Lmax_sub)/(Lmax-Lmax_sub));

			if((desiredPos+desplFromDesiredPos).y() > Lmax && desplFromDesiredPos.y() > 0) 
				desplFromDesiredPos.y() = desplFromDesiredPos.y() * (1-((desiredPos+desplFromDesiredPos).y()-Lmax_sub)/(Lmax-Lmax_sub));


			desiredPos = desiredPos+desplFromDesiredPos;

			glPointSize(15);
			glDisable(GL_LIGHTING);
			glColor3f(0,0,1);
			glBegin(GL_POINTS);
			glVertex3d(desiredPos.x(), desiredPos.y(), desiredPos.z());
			glColor3f(0,1,0);
			glVertex3d(newPoint.x(), newPoint.y(), newPoint.z());
			glEnd();
			glColor3f(1,1,1);
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