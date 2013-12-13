#include "SolverManager.h"
#include "AdriViewer.h"
#include "SolverSinusoidal.h"


SolverManager::SolverManager(void)
{
	solvers = map<int, vector<Solver*> > ();

	verletEnabled = vector<bool>();
	solversEnabled = vector<bool>();
	twistCorrectionEnabled = true;

	solverData = new SolverData();
}


SolverManager::~SolverManager(void)
{
}

Quaterniond SolverManager::computeTwist (joint* jt, Vector3d nLook, Vector3d restUp) {

	Vector3d axis(1,0,0);
	nLook.normalize();



	if (nLook.z() < -0.99) {
		if (nLook.x() < 0) {
			Quaterniond q;	q.setFromTwoVectors(nLook, Vector3d(1,0,0));
			q = Quaterniond::Identity().slerp(0.05, q);
			nLook = q._transformVector(nLook);
		} else {
			Quaterniond q;	q.setFromTwoVectors(nLook, Vector3d(-1,0,0));
			q = Quaterniond::Identity().slerp(0.05, q);
			nLook = q._transformVector(nLook);
		}
	}
	if (abs(nLook.dot(axis)) < 0.00005) return Quaterniond::Identity();

	Quaterniond fatherRotation = jt->father->rotation;
	Quaterniond fatherRotation2 = fatherRotation * jt->qOrient;
	//fatherRotation2 = jt->rotation;

	Vector3d u = jt->rotation._transformVector(jt->rRotation.inverse()._transformVector(Vector3d(0,1,0)));
	Vector3d v = Vector3d(0,1,0);

	Vector3d uprime = jt->rotation.inverse()._transformVector(u);
	Vector3d vprime = jt->rotation.inverse()._transformVector(v);
	Vector3d w = jt->qrot.inverse()._transformVector(uprime);
	Quaterniond newqrot;
	newqrot.setFromTwoVectors(w, vprime);
	//uprime = restUp;
	//Quaterniond newqrot;
	//newqrot.setFromTwoVectors(uprime, vprime);

	return newqrot;
}

void SolverManager::update (int sk, skeleton* s) {
	int n = solvers[sk].size();
	Solver* lastSolver = solvers[sk][solvers[sk].size()-1];
	Solver* firstSolver = solvers[sk][0];
	lastSolver->update();
	Chain* chain = lastSolver->outputs[sk];

	// Move root to initial joint
	Vector3d translation = chain->positions[0] - s->joints[0]->translation;
	s->joints[0]->addTranslation(translation.x(), translation.y(), translation.z());

	vector<double> jointTwist (chain->positions.size(), 0);
	int numTwisted = 5;
	Quaterniond removedTwist = Quaterniond::Identity();


	for (int i = 0; i < s->joints.size(); ++i) s->joints[i]->qOrient = s->joints[i]->restOrient;
	s->joints[0]->computeWorldPos();

	// First pass to compute twist. TO OPTIMIZE
	for (int i = 0; i < chain->positions.size()-1; ++i) {
		s->joints[0]->computeWorldPos();
		joint* jt = s->joints[i], * father, * child;

		Vector3d fatherPosition (0,0,0); 
		Quaterniond fatherRotation = Quaterniond::Identity(); 
		Quaterniond fatherTwist = Quaterniond::Identity();

		if(jt->father) {
			father = jt->father;
			fatherPosition = father->translation;
			fatherRotation = father->rotation;
			fatherTwist = father->twist;
		} else jt->translation = fatherPosition + fatherRotation._transformVector(jt->pos);

		if(jt->childs.size()>0)
			child = jt->childs[0];
		else break;

		//jt->translation = fatherPosition + fatherRotation._transformVector(jt->pos);
		Quaterniond fatherRotation2 = fatherRotation * jt->qOrient;
		child->translation = jt->translation + (fatherRotation2 * jt->qrot)._transformVector(child->pos);

		Vector3d u = child->translation - jt->translation;
		Vector3d v = chain->positions[i+1] - jt->translation;

		Vector3d uprime = fatherRotation2.inverse()._transformVector(u);
		Vector3d vprime = fatherRotation2.inverse()._transformVector(v);
		Vector3d w = jt->qrot.inverse()._transformVector(uprime);
		Quaterniond newqrot;
		newqrot.setFromTwoVectors(w, vprime);

		jt->qrot = newqrot;
		jt->rotation = fatherRotation * jt->qOrient * jt->qrot;

		// Twist correction
		if (i == chain->positions.size()-2 && brains[sk]->lookInit && twistCorrectionEnabled) {
			//Vector3d nLook = solverData->neck._transformVector(s->joints[19]->translation - s->joints[18]->translation).normalized();
			Vector3d nLook = solverData->neck._transformVector(chain->positions[19] - chain->positions[18]).normalized();

			// Twist correcter is the quaternion from u to v, where u and v are projections onto the plane with n = nLook
			// proj(v,n) = v - v.dot(n) * n
			Vector3d quatAxis = nLook;
			Vector3d u = jt->rotation._transformVector(jt->rRotation.inverse()._transformVector(Vector3d(0,1,0))).normalized();
			Vector3d v (0,1,0);
			u = (u - u.dot(quatAxis)*quatAxis).normalized();
			v = (v - v.dot(quatAxis)*quatAxis).normalized();
			u = jt->rotation.inverse()._transformVector(u);
			v = jt->rotation.inverse()._transformVector(v);
			//double w = u.dot(v);
			//Quaterniond twistCorrecter(w, quatAxis.x(), quatAxis.y(), quatAxis.z());
			Quaterniond twistCorrecter;	twistCorrecter.setFromTwoVectors(u, v);
			//twistCorrecter = Quaterniond(twistCorrecter.w(), quatAxis.x(), quatAxis.y(), quatAxis.z());
			//twistCorrecter.normalize();
			//if (u.isApprox(v), 0.001) twistCorrecter = Quaterniond::Identity();


			//Quaterniond twistQuat = computeTwist(jt, nLook, brains[sk]->restUpVector);
			Quaterniond twistQuat = twistCorrecter;

			Vector3d currentUp = jt->rotation._transformVector(brains[sk]->restUpVector);
			currentUp = jt->rotation._transformVector(jt->rRotation.inverse()._transformVector(Vector3d(0,1,0)));

			// Project currentUp to the plane formed by UP, FRONT and save the rotation from 0,1,0 to it
			Vector3d vProj = currentUp.dot(Vector3d(0,1,0))*Vector3d(0,1,0) + currentUp.dot(Vector3d(0,0,1))*Vector3d(0,0,1);
			vProj = nLook;
			vProj.x() = 0;
			vProj.normalize();
			Quaterniond savedQuat;	savedQuat.setFromTwoVectors(jt->rotation.inverse()._transformVector(Vector3d(0,1,0)), 
																jt->rotation.inverse()._transformVector(vProj));

			Vector3d pp;
			/*glDisable(GL_LIGHTING);
			glColor3f(1,1,0);
			glBegin(GL_LINES);																// DRAW CURRENT UP
			pp = jt->translation + nLook*100;
			glVertex3d(jt->translation.x(), jt->translation.y(), jt->translation.z());
			glVertex3d(pp.x(), pp.y(), pp.z());
			glEnd();

			glColor3f(1,0,0);
			glBegin(GL_LINES);																// DRAW CURRENT UP
			pp = jt->translation + u*100;
			glVertex3d(jt->translation.x(), jt->translation.y(), jt->translation.z());
			glVertex3d(pp.x(), pp.y(), pp.z());
			glEnd();

			glColor3f(0,0,1);
			glBegin(GL_LINES);																// DRAW CURRENT UP
			pp = jt->translation + v*100;
			glVertex3d(jt->translation.x(), jt->translation.y(), jt->translation.z());
			glVertex3d(pp.x(), pp.y(), pp.z());
			glEnd();*/

			jt->qrot = jt->qrot * twistQuat;
			//jt->qrot = twistQuat;
			s->joints[0]->computeWorldPos();

			/*glColor3f(0,0.5,1);
			pp = jt->translation + vProj*100;
			glBegin(GL_LINES);
			glVertex3d(jt->translation.x(), jt->translation.y(), jt->translation.z());
			glVertex3d(pp.x(), pp.y(), pp.z());
			glEnd();*/

			/*currentUp = jt->rotation._transformVector(brains[sk]->restUpVector);

			glDisable(GL_LIGHTING);
			glColor3f(0,1,0);
			glBegin(GL_LINES);
			pp = jt->translation + currentUp*100;
			glVertex3d(jt->translation.x(), jt->translation.y(), jt->translation.z());
			glVertex3d(pp.x(), pp.y(), pp.z());
			glEnd();*/

			//return;
			//s->joints[i+1]->qrot = twistQuat * s->joints[i+1]->qrot;

			Quaterniond twistQuat2 = twistQuat;
			twistQuat = jt->twist;
			Vector3d axis = jt->twist.vec().normalized();

			Quaterniond twistAux = jt->twist;
			Vector3d newV = jt->father->qrot._transformVector(twistAux.vec());
			twistAux = Quaterniond(twistAux.w(), newV.x(), newV.y(), newV.z());
			jt->qOrient = twistAux.inverse() * jt->qOrient;

			removedTwist = twistAux;

			s->joints[0]->computeWorldPos();

			if (removedTwist.isApprox(Quaterniond::Identity(), 0.0001)) return;

			for (int i = jointTwist.size()-2; i > jointTwist.size()-2 - numTwisted; --i) jointTwist[i] = 1.0 / numTwisted;

			// Smoothing
			for (int i = 0; i < 0; ++i) {
				vector<double> averages (jointTwist.size());
				double sum = 0;
				for (int j = 0; j < jointTwist.size()-1; ++j) {
					if (j == 0) averages[j] = (jointTwist[j] + jointTwist[j+1]) / 2.0;
					else if (j == jointTwist.size()-2) averages[j] = (jointTwist[j-1] + jointTwist[j]) / 2.0;
					else averages[j] = (jointTwist[j-1] + jointTwist[j+1]) / 2.0;
					sum += averages[j];
				}
				//for (int i = jointTwist.size()-1; i > jointTwist.size()-1 - numTwisted; --i) jointTwist[i] += (1.0 - sum) / numTwisted;
				for (int j = averages.size()-1; j > jointTwist.size()-1 - numTwisted; --j) averages[j] += (1.0 - sum) / numTwisted;
				for (int j = 0; j < jointTwist.size(); ++j) jointTwist[j] = averages[j];
			}

			double sum = 0;
			for (int j = 0; j < jointTwist.size(); ++j) sum += jointTwist[j];
			assert(sum == 1.0);
		}

		jt->rotation = fatherRotation * jt->qOrient * jt->qrot;
	}
	s->joints[0]->computeWorldPos();

	if (!brains[sk]->lookInit) {
		brains[sk]->lookInit = true;
		//brains[sk]->restUpVector = s->joints[18]->rotation.inverse()._transformVector(Vector3d(0,1,0));
		return;
	}
	
	if (!twistCorrectionEnabled) return;

	Quaterniond transformation = Quaterniond::Identity();
	for (int i = chain->positions.size()-2; i >= 0; --i) {
		if (jointTwist[i] < 0.05) continue;
		Quaterniond currentTwist = Quaterniond::Identity().slerp(jointTwist[i], removedTwist);
		joint* jt = s->joints[i];

		Vector3d v = currentTwist.vec();
		Vector3d newV = transformation._transformVector(v);
		currentTwist = Quaterniond(currentTwist.w(), newV.x(), newV.y(), newV.z());
		jt->qOrient = currentTwist * jt->qOrient;

		if (jt->father != 0) transformation = transformation * (jt->father->qrot * jt->qOrient);
	}

	s->joints[0]->computeWorldPos();
}