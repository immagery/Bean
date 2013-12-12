#include "SolverManager.h"
#include "AdriViewer.h"
#include "SolverSinusoidal.h"


SolverManager::SolverManager(void)
{
	solvers = map<int, vector<Solver*> > ();

	verletEnabled = vector<bool>();
	solversEnabled = vector<bool>();

	solverData = new SolverData();
}


SolverManager::~SolverManager(void)
{
}

Quaterniond SolverManager::computeTwist (joint* jt, Vector3d nLook) {
	Vector3d axis(1,0,0);
	if (nLook.x() <= 0) axis = Vector3d(-1,0,0);
	double projection = nLook.dot(axis);

	if (abs(projection) > 0.005) {
		if (nLook.z() < 0) projection = (1 - projection);

		double corrAngle = (M_PI/2) * abs(projection);
		if (nLook.z() < 0) corrAngle += (M_PI / 2);

		//Vector3d vectorCorrection (0, sin(corrAngle), -cos(corrAngle));
		Vector3d vectorCorrection (0, cos(corrAngle), -sin(corrAngle));
		Vector3d vc1 (0,0,-1);
		Vector3d vc2 = vectorCorrection;
		vc1 = vectorCorrection;
		vc2 = Vector3d (0,1,0);
	
		Quaterniond correction;	correction.setFromTwoVectors(vc1, vc2);
		if (nLook.x() < 0)		correction.setFromTwoVectors(vc2, vc1);

		return correction;
	}

	return Quaterniond::Identity();
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
	int numTwisted = 15;
	Quaterniond removedTwist = Quaterniond::Identity();

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

		//if (uprime.isApprox(vprime, 0.1)) continue;

		jt->qrot = newqrot;
		jt->rotation = fatherRotation * jt->qOrient * jt->qrot;

		// Twist correction
		if (i == chain->positions.size()-2) {
			//Vector3d nLook = solverData->neck._transformVector(s->joints[19]->translation - s->joints[18]->translation).normalized();
			Vector3d nLook = solverData->neck._transformVector(chain->positions[19] - chain->positions[18]).normalized();
			Quaterniond twistQuat = computeTwist(jt, nLook);
			
			double fullTwist = twistQuat.w();
			if (twistQuat.isApprox(Quaterniond::Identity(), 0.01)) continue;

			jt->qrot = jt->qrot * twistQuat;
			s->joints[0]->computeWorldPos();
			//s->joints[i+1]->qrot = twistQuat * s->joints[i+1]->qrot;

			Quaterniond twistQuat2 = twistQuat;
			twistQuat = jt->twist;
			Vector3d axis = jt->twist.vec().normalized();
			if (twistQuat.isApprox(Quaterniond::Identity(), 0.01)) continue;

			Quaterniond twistAux = jt->twist;
			Vector3d newV = jt->father->qrot._transformVector(twistAux.vec());
			twistAux = Quaterniond(twistAux.w(), newV.x(), newV.y(), newV.z());
			jt->qOrient = twistAux.inverse() * jt->qOrient;
			//jt->qrot = jt->qrot * jt->twist.inverse();

			removedTwist = twistAux;

			s->joints[0]->computeWorldPos();

			glDisable(GL_LIGHTING);
			glColor3f(0,1,1);
			glBegin(GL_LINES);
			Vector3d v = twistAux.vec();
			v = jt->father->rotation._transformVector(v);
			Vector3d p2 = jt->father->translation + v*50*twistAux.w();
			glVertex3d(jt->father->translation.x(), jt->father->translation.y(), jt->father->translation.z());
			glVertex3d(p2.x(), p2.y(), p2.z());
			glEnd();
			glEnable(GL_LIGHTING);

			for (int i = jointTwist.size()-2; i > jointTwist.size()-2 - numTwisted; --i) jointTwist[i] = 1.0 / numTwisted;

			// Smoothing
			for (int i = 0; i < 0; ++i) {
				double sum = 0;
				for (int j = 1; j < jointTwist.size()-1; ++j) {
					jointTwist[j] = (jointTwist[j-1] + jointTwist[j+1]) / 2;
					sum += jointTwist[j];
				}
				jointTwist[jointTwist.size()-1] = jointTwist[jointTwist.size()-2] / 2;
				for (int i = jointTwist.size()-1; i > jointTwist.size()-1 - numTwisted; --i) jointTwist[i] += (1.0 - sum) / numTwisted;
			}
		}

		jt->rotation = fatherRotation * jt->qOrient * jt->qrot;
	}
	s->joints[0]->computeWorldPos();

	Quaterniond transformation = Quaterniond::Identity();


	/*int i = chain->positions.size()-3;
	transformation = (s->joints[i]->qrot * s->joints[i+1]->qOrient);
	
	Vector3d v = removedTwist.vec();
	Vector3d newV = transformation._transformVector(v);
	removedTwist = Quaterniond(removedTwist.w(), newV.x(), newV.y(), newV.z());
	s->joints[i]->qOrient = removedTwist * s->joints[i]->qOrient;
	s->joints[0]->computeWorldPos();
	return;*/

	for (int i = chain->positions.size()-2; i >= 0; --i) {
		Quaterniond currentTwist = Quaterniond::Identity().slerp(jointTwist[i], removedTwist);
		joint* jt = s->joints[i];

		Vector3d v = currentTwist.vec();
		Vector3d newV = transformation._transformVector(v);
		currentTwist = Quaterniond(currentTwist.w(), newV.x(), newV.y(), newV.z());
		jt->qOrient = currentTwist * jt->qOrient;

		if (jt->father != 0) transformation = transformation * (jt->father->qrot * jt->qOrient);

		continue;

		/*if (jointTwist[i] < 0.05) continue;
		joint* jt = s->joints[i];
		joint* father, *child;*/

		/*Vector3d fatherPosition (0,0,0); 
		Quaterniond fatherRotation = Quaterniond::Identity(); 
		Quaterniond fatherTwist = Quaterniond::Identity();

		if(jt->father) {
			father = jt->father;
			fatherPosition = father->translation;
			fatherRotation = father->rotation;
		} 

		if(jt->childs.size()>0)
			child = jt->childs[0];
		else break;*/

		//Vector3d axis = (child->translation - jt->translation).normalized();
		//Vector3d axis2 = (fatherRotation * jt->qOrient).inverse()._transformVector(axis);
		//axis = (fatherRotation * jt->qOrient)._transformVector(axis);

		/*Vector3d axis = (jt->translation - father->translation);
		axis = fatherRotation.inverse()._transformVector(axis).normalized();*/

		//jt->father->qrot = Quaterniond(jointTwist[i], axis.x(), axis.y(), axis.z()).normalized() * jt->father->qrot;
		//jt->qrot = (Quaterniond(jointTwist[i], axis2.x(), axis2.y(), axis2.z()).normalized() * jt->qrot).normalized();
		//jt->father->rotation = jt->father->father->rotation * jt->father->qOrient * jt->father->qrot;
		//s->joints[0]->computeWorldPos();
		//jt->rotation = fatherRotation * jt->qOrient * jt->qrot;
	}

	s->joints[0]->computeWorldPos();
}

		/*if (i == chain->positions.size()-2) {
			Vector3d nLook = solverData->neck._transformVector(s->joints[19]->translation - s->joints[18]->translation).normalized();
			Quaterniond twistQuat = computeTwist(jt, nLook);
			
			double fullTwist = twistQuat.w();
			if (twistQuat.isApprox(Quaterniond::Identity(), 0.01)) continue;

			jt->qrot = jt->qrot * twistQuat;
			s->joints[0]->computeWorldPos();

			Quaterniond twistQuat2 = twistQuat;
			twistQuat = jt->twist;
			if (twistQuat.isApprox(Quaterniond::Identity(), 0.01)) continue;

			for (int i = jointTwist.size()-2; i > jointTwist.size()-2 - numTwisted; --i) jointTwist[i] = fullTwist / numTwisted;

			// Smoothing
			for (int i = 0; i < 0; ++i) {
				double sum = 0;
				for (int j = 1; j < jointTwist.size()-1; ++j) {
					jointTwist[j] = (jointTwist[j-1] + jointTwist[j+1]) / 2;
					sum += jointTwist[j];
				}
				jointTwist[jointTwist.size()-1] = jointTwist[jointTwist.size()-2] / 2;
				for (int i = jointTwist.size()-1; i > jointTwist.size()-1 - numTwisted; --i) jointTwist[i] += (fullTwist - sum) / numTwisted;
			}	
		}*/