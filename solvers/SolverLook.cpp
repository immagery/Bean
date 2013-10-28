#include "SolverLook.h"
#include "AdriViewer.h"
#include "Eigen\Dense"


SolverLook::SolverLook() {

}

vector<pair<int,Quaternion<double> > > SolverLook::solve(double time) {
	vector<pair<int,Quaternion<double> > > result(chain.size());
	for (int i = 0; i < result.size(); ++i) 
		result[i] = pair<int,Quaternion<double> > (chain[i].second, Quaternion<double>(1,0,0,0));

	for (int i = 0; i < chain.size(); ++i) {
		Point3d targetVector = (lookPoint - chain[i].first->getWorldPosition()).normalized();
		Quaternion<double> qt = chain[i].first->qrot; 
		Eigen::Quaterniond qq(qt.W(), qt.X(), qt.Y(), qt.Z());
		qq.normalize();
		//Eigen::Vector3d vv1 = qq._transformVector(Eigen::Vector3d(lookVector.X(), lookVector.Y(), lookVector.Z()));
		Eigen::Vector4f vv1 = chain[i].first->W * Eigen::Vector4f(lookVector.X(), lookVector.Y(), lookVector.Z(), 1);
		Eigen::Vector4f pe2 = chain[i].first->W * (Eigen::Vector4f(322, -400, 0, 1));
		Eigen::Vector4f pe3 = chain[i].first->W * (Eigen::Vector4f(0, 0, 0, 1));
			Point3d p2(pe2.x(), pe2.y(), pe2.z());
			Point3d p3(pe3.x(), pe3.y(), pe3.z());
		//Eigen::Vector3d vv2 = qq._transformVector(Eigen::Vector3d(targetVector.X(), targetVector.Y(), targetVector.Z()));
		//Point3d v1 = Point3d(vv1.x(), vv1.y(), vv1.z()).normalized();
		Point3d v1 = p2 - p3;
		Point3d v2 = (lookPoint - chain[i].first->getWorldPosition());
		//Point3d v2 = Point3d(vv2.x(), vv2.y(), vv2.z()).normalized();
		Point3d v1L = v1*5;
		Point3d v2L = v2*5;
		Point3d v3L = lookVector*5;
		Point3d v3 = lookVector;

		double cos = (v2.dot(v3) / (v2.Norm() * v3.Norm()));
		double angle = acos(cos) * 180 / M_PI;
		if (abs(angle) > 45) {
			return result;
		}

		glDisable(GL_LIGHTING);
		glColor3f(1,0,0);
		glBegin(GL_LINES);
		Point3d p = chain[i].first->getWorldPosition();
		glVertex3f(p.X(), p.Y(), p.Z());
		glVertex3f((p+v1L).X(), (p+v1L).Y(), (p+v1L).Z());
		glEnd();
		glColor3f(0,1,0);
		glBegin(GL_LINES);
		glVertex3f(p.X(), p.Y(), p.Z());
		glVertex3f((p+v2L).X(), (p+v2L).Y(), (p+v2L).Z());
		glEnd();
		glColor3f(0,1,1);
		glBegin(GL_LINES);
		glVertex3f(p.X(), p.Y(), p.Z());
		glVertex3f((p+v3L).X(), (p+v3L).Y(), (p+v3L).Z());
		glEnd();
		glEnable(GL_LIGHTING);

		v1.Normalize();
		v2.Normalize();

		if ((v1-v2).Norm() < 0.01) continue;


		vcg::Quaternion<double> q;
		q.FromAxis(acos(v1.dot(v2) / (v1.Norm() * v2.Norm())), v1^v2);
		q.Normalize();
		
		//result[i] = pair<int, Point3d> (chain[i].second, Point3d(-ry,-rz,-rx));
		result[i] = pair<int, Quaternion<double> > (chain[i].second, q);
	}

	return result;
}