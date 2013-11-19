#include "SolverSinusoidal.h"
#include "AdriViewer.h"

vector<pair<int,Eigen::Vector3d > > SolverSinusoidal::solve(double time) {
	vector<pair<int,Eigen::Vector3d > > result;
	vector<Eigen::Vector3d> nextPositions(chain.size());

	// Fill vectors with pre and post positions
	for (int i = 0; i < nextPositions.size(); ++i) {
		double mAmp = amplitude * ((longitude - i) / longitude);
		//mAmp = amplitude;
		mAmp = amplitude * (i / longitude);
		double inc = (mAmp * sin(freq*time - i/longitude)) - (mAmp * sin(freq*lastTime - i/longitude))    ;

		Eigen::Vector3d oldPos = restPositions[i];
		Eigen::Vector3d nextPos;

		if (dimension == 0)			nextPos = Vector3d(inc,0,0);
		else if (dimension == 1)	nextPos = Vector3d(0,inc,0);
		else if (dimension == 2)	nextPos = Vector3d(0,0,inc);

		result.push_back(pair<int, Eigen::Vector3d>(chain[i].first, nextPos));
	}

	lastTime = time;
	return result;
}

void SolverSinusoidal::setPositions() {
	restPositions = vector<Eigen::Vector3d> (chain.size());
	for (int i = 0; i < restPositions.size(); ++i)
		restPositions[i] = chain[i].second;
}
