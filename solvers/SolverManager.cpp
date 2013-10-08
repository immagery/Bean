#include "SolverManager.h"


SolverManager::SolverManager(void)
{
}


SolverManager::~SolverManager(void)
{
}

vector<Point3d> SolverManager::computeSolvers(int frame, const vector<skeleton*>& skeletons) {
	vector<Point3d> finalPositions(skeletons[0]->joints.size());
	for (int i = 0; i < finalPositions.size(); ++i) finalPositions[i] = Point3d(0,0,0);

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
