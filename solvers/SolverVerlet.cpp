#include "SolverVerlet.h"


SolverVerlet::SolverVerlet(void)
{

	stiffness = 1;
	g = 9.8;
}


SolverVerlet::~SolverVerlet(void)
{
}

void SolverVerlet::bake(int maxFrames, double deltaTimePerFrame) {
	bakedPositions.resize(maxFrames);
	deltaTimePerFrame /= 1;

	for (int frame = 0; frame < bakedPositions.size(); ++frame) {

		bakedPositions[frame] = (vector<Point3d> (currentPositions.size()));



	}
}

vector<pair<int,Point3d> > SolverVerlet::solve(double time) {
	vector<pair<int,Point3d> > result(chain.size());
	double deltaTime = (time - lastTime);

	// First, solve constraints (distances between links) ( should be repeated k times )
	for (int i = 0; i < restPositions.size()-1; ++i) {
		Point3d restDistance = restPositions[i+1] - restPositions[i];
		Point3d currentDist = currentPositions[i+1] - currentPositions[i];
		double rd = restDistance.Norm();
		double cd = currentDist.Norm();
		double diff = (rd - cd) / rd;
		double scalarP1 = 0.5 * stiffness;
		double scalarP2 = stiffness - scalarP1;
		Point3d currentI = currentDist * scalarP1 * diff;
		currentPositions[i] -= currentDist * scalarP1 * diff * deltaTime;
		currentPositions[i+1] += currentDist * scalarP2 * diff * deltaTime;
	}

	double tsq = deltaTime*deltaTime;

	// Then update physics of each point
	for (int i = 0; i < chain.size(); ++i) {
		Point3d velocity = currentPositions[i] - lastPositions[i];						// velocity = inertia
		Point3d nextPos = currentPositions[i] + velocity + Point3d(0,-g,0) * tsq;		// apply gravity

		if (i > 0) {
			printf("Current pos of 2nd joint: %f %f %f\n", nextPos.X(), nextPos.Y(), nextPos.Z());
			
			// Which angle should we rotate the previous joint in order to position this one in nextPos?
			double tanX = currentPositions[i-1].X() - lastPositions[i-1].X();
			double tanY = currentPositions[i-1].Y() - lastPositions[i-1].Y();
			double tanZ = currentPositions[i-1].Z() - lastPositions[i-1].Z();

			double angleX = atan(tanX);
			double angleY = atan(tanY);
			double angleZ = atan(tanZ);

			result[i-1] = pair<int, Point3d> (chain[i-1].second, Point3d(angleX, angleY, angleZ));
			Point3d finalRot = result[i-1].second * 180 / 3.141592;
			//result[i-1].second = finalRot - chain[i-1].first->rot;
		}

		lastPositions[i] = currentPositions[i];
		currentPositions[i] = nextPos;

	}

	lastTime = time;
	result[result.size()-1] = pair<int, Point3d> (chain[result.size()-1].second, Point3d(0,0,0));
	return result;
}

void SolverVerlet::setPositions() {
	restPositions.resize(chain.size());
	lastPositions.resize(chain.size());
	currentPositions.resize(chain.size());

	lastTime = 0;
	for (int i = 0; i < lastPositions.size(); ++i) {
		Point3d pos = chain[i].first->worldPosition;
		printf("Initial pos of %dth joint: %f %f %f\n", i, pos.X(), pos.Y(), pos.Z());
		lastPositions[i] = restPositions[i] = currentPositions[i] = pos;
	}
}