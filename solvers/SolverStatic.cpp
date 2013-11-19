#include "SolverStatic.h"

SolverStatic::SolverStatic(void)
{

}


SolverStatic::~SolverStatic(void)
{
}

void SolverStatic::setStatic() {
	staticPositions.resize(chain.size());
	for (int i = 0; i < chain.size(); ++i) {
		staticPositions[i] = chain[i].second;
	}
}

vector<pair<int,Eigen::Vector3d > > SolverStatic::solve(double time) {
	vector<pair<int,Eigen::Vector3d > > result(chain.size());
	for (int i = 0; i < result.size(); ++i) 
		result[i] = pair<int,Eigen::Vector3d > (chain[i].first, staticPositions[i]);
	return result;
}