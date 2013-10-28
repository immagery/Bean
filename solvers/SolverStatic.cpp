#include "SolverStatic.h"

SolverStatic::SolverStatic(void)
{

}


SolverStatic::~SolverStatic(void)
{
}

void SolverStatic::setStatic() {
	staticAngles.resize(chain.size());
	for (int i = 0; i < chain.size(); ++i) staticAngles[i] = chain[i].first->qrot;
}

vector<pair<int,Quaternion<double> > > SolverStatic::solve(double time) {
	vector<pair<int,Quaternion<double> > > result(chain.size());
	for (int i = 0; i < result.size(); ++i) 
		result[i] = pair<int,Quaternion<double> > (chain[i].second, Quaternion<double>(1,0,0,0));
	return result;

	/*for (int i = 0; i < chain.size(); ++i) {
		Quaternion<double> current = chain[i].first->qrot;
		Quaternion<double> target = staticAngles[i];
		Quaternion<double> final = current.Inverse() * target;
		final.Normalize();
		double rx, ry, rz;	final.ToEulerAngles(rx,ry,rz);
		rx = (rx *360)/(M_PI*2);
		ry = (ry *360)/(M_PI*2);
		rz = (rz *360)/(M_PI*2);

		// COMMENTS
		result[i].first = chain[i].second;
		Quaternion<double> target = staticAngles[i];
		Quaternion<double> current = chain[i].first->qrot;
		Point3d axis;	double phi;	current.ToAxis(phi, axis);
		Point3d lookV = Point3d(0,500,300) - chain[i].first->getWorldPosition();

		Point3d v1 = axis;	Point3d v2 = lookV;

		vcg::Quaternion<double> q;
		q.FromAxis(acos(v1.dot(v2) / (v1.Norm() * v2.Norm())), v1^v2);
		q.Normalize();
		double rx, ry, rz;	q.ToEulerAngles(rx,ry,rz);
		rx = (rx *360)/(M_PI*2);
		ry = (ry *360)/(M_PI*2);
		rz = (rz *360)/(M_PI*2);

		Quaternion<double> final = current.Inverse() * target;
		double rx, ry, rz;	final.ToEulerAngles(rx,ry,rz);
		rx = (rx *360)/(M_PI*2);
		ry = (ry *360)/(M_PI*2);
		rz = (rz *360)/(M_PI*2);
		// END

		
		result[i] = pair<int, Point3d> (chain[i].second, Point3d(ry,rz,rx));
	}
	return result;*/
}