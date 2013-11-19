#include "SolverLook.h"
#include "AdriViewer.h"


SolverLook::SolverLook() {

}

vector<pair<int,Eigen::Vector3d > > SolverLook::solve(double time) {
	vector<pair<int,Vector3d > > result(1);

	Eigen::Vector3d v1 = chain[1].second - chain[0].second;
	Eigen::Vector3d v2 = lookPoint - chain[0].second;
	Vector3d newPos = chain[0].second + (v2.normalized() * v1.norm());

	result[0] = pair<int,Vector3d> (chain[1].first, newPos - chain[1].second);

	return result;
}
