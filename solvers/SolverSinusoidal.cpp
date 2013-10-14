#include "SolverSinusoidal.h"


SolverSinusoidal::SolverSinusoidal(void)
{
}

SolverSinusoidal::SolverSinusoidal(double a, double f, double ph) {
	amplitude = a;
	freq = f;
	phase = ph;
}

vector<pair<int,Point3d> > SolverSinusoidal::solve(double time) {
	vector<pair<int,Point3d> > result(chain.size());
	for (int i = 0; i < chain.size()-1; ++i) {
		result[i].first = chain[i].second;
		//Point3d pos = chain[i].first->rot;
		double newPos = amplitude * sin(freq*time - i + phase);
		Point3d dist = distances[i];
		double d = dist.Norm();
		d = 1;
		double tanAngle = newPos / d;	// cateto contiguo = 1
		double rot = atan(tanAngle);
		rot = rot * 180 / 3.141592;

		double rx, ry, rz;
		chain[i].first->qrot.ToEulerAngles(rx,ry,rz);
		rx = (rx*360)/(M_PI*2);
		ry = (ry*360)/(M_PI*2);
		rz = (rz*360)/(M_PI*2);


		if (dimension == 0) {
			rot = rot - rx;
			result[i].second = Point3d(rot, 0, 0);
		}
		if (dimension == 1) {
			rot = rot - ry;
			result[i].second = Point3d(0, rot, 0);
		}
		if (dimension == 2) {
			rot = rot - rz;
			result[i].second = Point3d(0, 0, rot);
		}
	}
	result[result.size()-1].second = Point3d(0,0,0);
	return result;
}


SolverSinusoidal::~SolverSinusoidal(void)
{
}
