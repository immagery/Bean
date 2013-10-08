#include "Particles.h"

#include <utils/utilGL.h>
#include <DataStructures/Geometry.h> 


Particles::Particles(void)
{
	lastTime = 0;
	graph = new SurfaceGraph();
	int n = 5;

	stiffness = 1;
	g = -9.8;
	xvalue = 0;

	graph->nodes.resize(n);
	for (int i = 0; i < n; ++i) {
		graph->nodes[i] = new GraphNode(i);
		graph->nodes[i]->position = Point3d(0,5*i,0.3*i);
		if (i > 0) {
			graph->nodes[i]->connections.push_back(graph->nodes[i-1]);
			graph->nodes[i-1]->connections.push_back(graph->nodes[i]);
		}
		
		restPositions.push_back(graph->nodes[i]->position);
		currentPositions.push_back(graph->nodes[i]->position);
		lastPositions.push_back(graph->nodes[i]->position);
	}
}


Particles::~Particles(void)
{
}

void Particles::drawFunc(int frame) {
	glColor3f(1.0, 0, 0);
	if (frame*100 >= positions.size()) return;
	vector<Point3d> pos = positions[frame*100];
	glPointSize(8);
	glBegin(GL_POINTS);
	for (int i = 0; i < graph->nodes.size(); ++i) {
		glVertex3d(pos[i].X(), pos[i].Y(), pos[i].Z());
	}
	glEnd();
	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	for (int i = 0; i < graph->nodes.size(); ++i) {
		if (i < graph->nodes.size()-1) {
			glVertex3d(pos[i].X(), pos[i].Y(), pos[i].Z());
			glVertex3d(pos[i+1].X(), pos[i+1].Y(), pos[i+1].Z());
		}
		/*for (int j = 0; j < graph->nodes[i]->connections.size(); ++j) {
			glVertex3d(pos[i].X(), pos[i].Y(), pos[i].Z());
			glVertex3d(pos[j].X(), pos[j].Y(), pos[j].Z());
		}*/
	}
	glEnd();
}

void Particles::drawFunc() {
	glColor3f(1.0, 0, 0);
	vector<Point3d> pos = currentPositions;
	glPointSize(8);
	glBegin(GL_POINTS);
	for (int i = 0; i < graph->nodes.size(); ++i) {
		glVertex3d(pos[i].X(), pos[i].Y(), pos[i].Z());
	}
	glEnd();
	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	for (int i = 0; i < graph->nodes.size(); ++i) {
		if (i < graph->nodes.size()-1) {
			glVertex3d(pos[i].X(), pos[i].Y(), pos[i].Z());
			glVertex3d(pos[i+1].X(), pos[i+1].Y(), pos[i+1].Z());
		}
		/*for (int j = 0; j < graph->nodes[i]->connections.size(); ++j) {
			glVertex3d(pos[i].X(), pos[i].Y(), pos[i].Z());
			glVertex3d(pos[j].X(), pos[j].Y(), pos[j].Z());
		}*/
	}
	glEnd();
}

void Particles::solve(double time) {
	double deltaTime = time - lastTime;
	double tsq = deltaTime * deltaTime;
	lastTime = time;

	int relaxNumber = 20;
	int neighbourDepth = 5;
	for (int k = 0; k < relaxNumber; ++k) {

			for (int i = 0; i < currentPositions.size(); ++i) {

				// Distance constraints
				double ks, kd;
				ks = 50;
				kd = 1;
				for (int j = i-neighbourDepth; j <= i+neighbourDepth; ++j) {
					/*if (j < 0 || j >= currentPositions.size() || j == i) continue;
					Point3d restDistance = restPositions[i] - restPositions[j];
					Point3d currentDist = currentPositions[i] - currentPositions[j];
					double diff = (restDistance.Norm() - currentDist.Norm()) / restDistance.Norm();
					double scalarP1 = 0.1;		
					double scalarP2 = 0.1;		// masses are 1
					diff = currentDist.Norm() - restDistance.Norm();
					Point3d delta1 = -(currentDist / restDistance.Norm()) * (scalarP1 * diff);
					Point3d delta2 = -delta1;
					currentPositions[i] += delta1 * deltaTime;
					currentPositions[j] += delta2 * deltaTime;
					currentPositions[0] = Point3d(xvalue,0,0);*/

					if (j < 0 || j >= currentPositions.size() || j == i) continue;
					Point3d restDistance = restPositions[i] - restPositions[j];
					Point3d currentDist = currentPositions[i] - currentPositions[j];
					double diff = currentDist.Norm() - restDistance.Norm();
					Point3d delta1 = currentDist / currentDist.Norm() * ks * diff;
					Point3d delta2 = - delta1;
					Point3d vel1 = (currentPositions[i] - lastPositions[i]);
					Point3d vel2 = (currentPositions[j] - lastPositions[j]);
					double v = (vel2 - vel1).dot(currentDist.normalized());
					Point3d damp1 = currentDist / currentDist.Norm() * kd * v;
					Point3d damp2 = - damp1;
					currentPositions[i] -= (delta1+damp1)*deltaTime;
					currentPositions[j] -= (delta2+damp2)*deltaTime;
					currentPositions[0] = Point3d(xvalue,0,0);
				} // end of distance constraints

				// Angle constraints, only applicable if node has a neighbour in each side
				if (i > 0 && i < currentPositions.size() - 1) {
					Point3d v1 = currentPositions[i] - currentPositions[i-1];
					Point3d v2 = currentPositions[i+1] - currentPositions[i];
					double dotProd = v1.dot(v2);
					double norms = (v1.Norm() * v2.Norm());
					double cosPhi = dotProd / norms;
					if (cosPhi > 1) cosPhi = 1;
					if (cosPhi < -1) cosPhi = -1;
					double angle = acos(cosPhi);
					angle = angle * 180 / 3.141592;
					angle = 180 - angle;

					// If the angle is not in a certain range of the original, then move it!
					v1 = restPositions[i] - restPositions[i-1];
					v2 = restPositions[i+1] - restPositions[i];
					dotProd = v1.dot(v2);
					cosPhi = dotProd / (v1.Norm() * v2.Norm());
					if (cosPhi > 1) cosPhi = 1;
					if (cosPhi < -1) cosPhi = -1;
					double restAngle = acos(cosPhi);
					restAngle = restAngle * 180 / 3.141592;
					restAngle = 180 - restAngle;

					if (abs(restAngle - angle) > 5) {
						Point3d deltaPos = (restPositions[i+1] - restPositions[i]) + currentPositions[i] - currentPositions[i+1];
						currentPositions[i+1] += deltaPos * deltaTime;
					}
				} // end of angle constraints


			}

			// "Angle" restriction
			//Point3d originalVector = restPositions[1] - restPositions[0];
			//Point3d currentVector = currentPositions[1] - currentPositions[0];
			//currentPositions[1] += (originalVector - currentVector) * deltaTime;
		}
		
		for (int i = 0; i < currentPositions.size(); ++i) {
			Point3d velocity = (currentPositions[i] - lastPositions[i]);						// velocity = inertia
			Point3d nextPos = currentPositions[i] + velocity + Point3d(0,g,0) * tsq;		// apply gravit
			lastPositions[i] = currentPositions[i];
			currentPositions[i] = nextPos;
		}
}

void Particles::bake(int maxFrames, double deltaTimePerFrame) {
	int stepsPerFrame = 100;
	positions.resize(maxFrames*stepsPerFrame);
	deltaTimePerFrame /= stepsPerFrame;

	for (int frame = 0; frame < positions.size(); ++frame) {

		positions[frame] = (vector<Point3d> (currentPositions.size()));

		int relaxNumber = 24;
		int neighbourDepth = 2;
		for (int k = 0; k < relaxNumber; ++k) {
			for (int i = 0; i < currentPositions.size(); ++i) {
				// Calculate distance to its neighbours
				for (int j = i-neighbourDepth; j <= i+neighbourDepth; ++j) {
					if (j < 0 || j >= currentPositions.size() || j == i) continue;
					Point3d restDistance = restPositions[i] - restPositions[j];
					Point3d currentDist = currentPositions[i] - currentPositions[j];
					double diff = (restDistance.Norm() - currentDist.Norm()) / restDistance.Norm();
					double scalarP1 = 0.5;		
					double scalarP2 = 0.5;		// masses are 1
					diff = currentDist.Norm() - restDistance.Norm();
					Point3d delta1 = -(currentDist / restDistance.Norm()) * (scalarP1 * diff);
					Point3d delta2 = -delta1;
					//currentPositions[i] -= currentDist * scalarP1 * diff * deltaTimePerFrame;
					//currentPositions[j] += currentDist * scalarP2 * diff * deltaTimePerFrame;
					currentPositions[i] += delta1 * deltaTimePerFrame;
					currentPositions[j] += delta2 * deltaTimePerFrame;
					currentPositions[0] = Point3d(frame / 10000.0,0,0);
					//currentPositions[0] = Point3d(0,0,0);
				}

				if (i == 1) {
					Point3d originalVector = restPositions[1] - restPositions[0];
					Point3d currentVector = currentPositions[1] - currentPositions[0];
					currentPositions[1] += (originalVector - currentVector) * deltaTimePerFrame;
				}
			}
		}

		double tsq = deltaTimePerFrame * deltaTimePerFrame;

		for (int i = 0; i < currentPositions.size(); ++i) {
			Point3d velocity = (currentPositions[i] - lastPositions[i]);						// velocity = inertia
			Point3d nextPos = currentPositions[i] + velocity + Point3d(0,g,0) * tsq;		// apply gravit
			lastPositions[i] = currentPositions[i];
			currentPositions[i] = nextPos;

			positions[frame][i] = nextPos;

		}

	}

	currentPositions.clear();
	lastPositions.clear();
	for (int i = 0; i < restPositions.size(); ++i) {
		currentPositions.push_back(restPositions[i]);
		lastPositions.push_back(restPositions[i]);
	}

}