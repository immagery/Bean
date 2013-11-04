#include "SolverSinusoidal.h"
#include "AdriViewer.h"


SolverSinusoidal::SolverSinusoidal(void)
{
}

SolverSinusoidal::SolverSinusoidal(double a, double f, double ph) {
	amplitude = a;
	freq = f;
	phase = ph;
}

vector<pair<int,Eigen::Quaternion<double> > > SolverSinusoidal::solve(double time) {
	vector<pair<int,Eigen::Quaternion<double> > > result;

	vector<vcg::Point3d> positions(chain.size());
	/*vector<Point3d> currentPositions(chain.size());
	for (int i = 0; i < positions.size(); ++i) {
		positions[i] = currentPositions[i] = restPositions[i];
		if (i == 0) continue;
		double inc = amplitude * sin(freq * time - i + phase);
		if (dimension == 0) positions[i] += Point3d(inc,0,0);
		else if (dimension == 1) positions[i] += Point3d(0,inc,0);
		else if (dimension == 2) positions[i] += Point3d(0,0,inc);
	}

	glColor3f(1,0,0);
	glBegin(GL_LINES);
	for (int i = 0; i < positions.size() - 1; ++i) {
		glVertex3d(positions[i].X(), positions[i].Y(), positions[i].Z());
		glVertex3d(positions[i+1].X(), positions[i+1].Y(), positions[i+1].Z());
	}
	glEnd();*/

	/*Point3d increment(0,0,0);
	for (int i = 0; i < chain.size()-1; ++i) {
		Point3d v1 = chain[i+1].first->getWorldPosition() + increment - positions[i];
		Point3d v2 = positions[i+1] - positions[i];
		v1.Normalize();
		v2.Normalize();
		if ((v1-v2).Norm() < 0.0001)
			continue;
		vcg::Quaternion<double> q;
		q.FromAxis(acos(v1.dot(v2) / (v1.Norm() * v2.Norm())), v1^v2);
		q.Normalize();
		chain[i].first->addRotation(q);
		chain[i].first->computeWorldPos();
		//result.push_back(pair<int, Quaternion<double> > (chain[i].second, q));
		//if (result.size() == 1) return result;
	}
	return result;*/

	for (int i = 0; i < chain.size()-1; ++i) {
		double inc = amplitude * sin(freq*time - (i+1) + phase);
		vcg::Point3d oldPos = chain[i+1].first->getWorldPosition();
		vcg::Point3d nextPos;
		
		if (dimension == 0) nextPos = oldPos + vcg::Point3d(inc,0,0);
		else if (dimension == 1) nextPos = oldPos + vcg::Point3d(0,inc,0);
		else if (dimension == 2) nextPos = oldPos + vcg::Point3d(0,0,inc);

		vcg::Point3d v1 = oldPos - chain[i].first->getWorldPosition();
		vcg::Point3d v2 = nextPos - chain[i].first->getWorldPosition();
		//Point3d v12 = oldPos - currentPos;
		//Point3d v22 = nextPos - currentPos;
		//currentPos = nextPos;

		v1.Normalize();
		v2.Normalize();
		if ((v1-v2).Norm() < 0.0001) { 
			continue;
		}
		Eigen::Quaternion<double> q (acos(v1.dot(v2) / (v1.Norm() * v2.Norm())), (v1^v2).X(), (v1^v2).Y(), (v1^v2).Z());
		q.normalize();
		//q.FromAxis(acos(v1.dot(v2) / (v1.Norm() * v2.Norm())), v1^v2);
		//q.Normalize();
		//double rx, ry, rz;	q.ToEulerAngles(rx,ry,rz);
		result.push_back(pair<int, Eigen::Quaternion<double> > (chain[i].second, q));
	}
	return result;
}


SolverSinusoidal::~SolverSinusoidal(void)
{
}
