#include "SolverSinusoidal.h"
#include "AdriViewer.h"

vector<pair<int,Eigen::Quaternion<double> > > SolverSinusoidal::solve(double time) {
	vector<pair<int,Eigen::Quaternion<double> > > result;
	vector<Eigen::Vector3d> originalPositions(chain.size());
	vector<Eigen::Vector3d> nextPositions(chain.size());

	// Fill vectors with pre and post positions
	for (int i = 1; i < nextPositions.size(); ++i) {
		double inc = amplitude * sin(freq*time - (i+1) + phase);
		Eigen::Vector3d oldPos = chain[i+1].first->getWorldPosition();
		Eigen::Vector3d nextPos;
		
		if (dimension == 0)			nextPos = oldPos + Eigen::Vector3d(inc,0,0);
		else if (dimension == 1)	nextPos = oldPos + Eigen::Vector3d(0,inc,0);
		else if (dimension == 2)	nextPos = oldPos + Eigen::Vector3d(0,0,inc);

		nextPositions[i] = nextPos;
		originalPositions[i] = oldPos;
	}

	for (int i = 1; i < chain.size(); ++i) {

		Eigen::Vector3d v1 = originalPositions[i+1] - originalPositions[i];
		Eigen::Vector3d v2 = nextPositions[i+1] - originalPositions[i];

		// Check for bad cases? TODO

		Eigen::Quaternion<double> q;
		q.setFromTwoVectors(v1,v2);
		result.push_back(pair<int, Eigen::Quaternion<double> > (chain[i].second, q));
	}
	return result;
}