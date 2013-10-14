#include "SolverManager.h"


SolverManager::SolverManager(void)
{
	hasVerlet = false;
	oscillation = false;
}


SolverManager::~SolverManager(void)
{
}

vector<Point3d> SolverManager::computeSolvers(int frame, int animationPeriod, const vector<skeleton*>& skeletons) {

	vector<Point3d> finalPositions(skeletons[0]->joints.size());
	for (int i = 0; i < finalPositions.size(); ++i) finalPositions[i] = Point3d(0,0,0);

	if (!oscillation) return finalPositions;

	for (int i = 0; i < solvers.size(); ++i) {
		Solver* s = solvers[i];
		vector<pair<int, Point3d> > solverPos = s->solve(frame/24.0);
		for (int j = 0; j < solverPos.size(); ++j) {
			int id = solverPos[j].first;
			finalPositions[id] += solverPos[j].second;
		}
	}

	return finalPositions;
}

vector<Point3d> SolverManager::computeVerlet(int frame, int animationPeriod, const vector<skeleton*>& skeletons) {
	vector<Point3d> finalPositions(skeletons[0]->joints.size());
	for (int i = 0; i < finalPositions.size(); ++i) finalPositions[i] = Point3d(0,0,0);

	if (!hasVerlet) return finalPositions;

	vector<Point3d> lastFramePositions = verlet->currentPositions;

	verlet->chain[0].first->computeWorldPos();
	verlet->currentPositions[0] = verlet->chain[0].first->getWorldPosition();

	double fps = 1.0/animationPeriod*1000;
	double currentTime = (double)frame/fps;
	int numReps = 20;
	for (int k = 0; k < numReps-1; ++k) {
		verlet->solve(currentTime + ((double)k / numReps)*animationPeriod/1000.0);
	}

	vector<pair<int,Point3d> > positions = verlet->solve(currentTime + ((double)(numReps-1) / numReps)*animationPeriod/1000.0);
	for (int i = 0; i < positions.size()-1; ++i) {

		int currentID = positions[i].first;
		int nextID = positions[i+1].first;

		Point3d v1 = (lastFramePositions[i+1] - lastFramePositions[i]);
		Point3d v2 = positions[i+1].second - positions[i].second;

		bool vectorsTooSimilar = (v1-v2).Norm() < 0.05;
		bool aVectorIsZero = (v1.Norm() < 0.0005 || v2.Norm() < 0.0005);

		if (vectorsTooSimilar || aVectorIsZero) {		// numerical errors!
			finalPositions[currentID] = Point3d(0,0,0);
			continue;
		}

		vcg::Quaternion<double> q;
		q.FromAxis(acos(v1.dot(v2) / (v1.Norm() * v2.Norm())), v1^v2);
		q.Normalize();
		double rx, ry, rz;	q.ToEulerAngles(rx,ry,rz);
		rx = (rx *360)/(M_PI*2);
		ry = (ry *360)/(M_PI*2);
		rz = (rz *360)/(M_PI*2);
		finalPositions[currentID] = Point3d(-ry/10.0,-rz/10.0,-rx/10.0);
	}

	return finalPositions;
}