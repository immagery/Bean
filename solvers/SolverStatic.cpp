#include "SolverStatic.h"

SolverStatic::SolverStatic(void)
{

}


SolverStatic::~SolverStatic(void)
{
}

void SolverStatic::setStatic() {
	staticAngles.resize(chain.size());
	//for (int i = 0; i < chain.size(); ++i) staticAngles[i] = chain[i].first->rot;
}

vector<pair<int,Point3d> > SolverStatic::solve(double time) {
	vector<pair<int,Point3d> > result(chain.size());
	for (int i = 0; i < chain.size(); ++i) {
		result[i].first = chain[i].second;
		//result[i].second = staticAngles[i] - chain[i].first->rot;		// return the difference between desired angle and current one
	}
	return result;
}