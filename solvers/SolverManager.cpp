#include "SolverManager.h"
#include "AdriViewer.h"
#include "SolverSinusoidal.h"


SolverManager::SolverManager(void)
{
	solversChainRef.clear();
	verletEnabled.clear();
	solversEnabled.clear();

	solverData = new SolverData();

	numTwisted = 3;
	smoothingIterations = 0;
	twistCorrectionEnabled = true;
}

SolverManager::~SolverManager(void)
{
}

void SolverManager::draw() 
{
	GLUquadricObj *quadric;
	quadric = gluNewQuadric();
	gluQuadricDrawStyle(quadric, GLU_LINE );
	glDisable(GL_LIGHTING);

	for (int characterId = 0; characterId < solversChainRef.size(); ++characterId) 
	{
		int solversCount = solversChainRef[characterId].size();
		if(solversCount > 0)
		{
			Solver* lastSolver = solversChainRef[characterId].back();
			Chain* chain = lastSolver->outputs[0];

			glColor3f(1,0,0);
			for (int i = 0; i < chain->positions.size(); ++i)
			{
				Vector3d p = chain->positions[i];
				glPushMatrix();
				glTranslated(p.x(), p.y(), p.z());
				gluSphere(quadric,3,8,8);
				glPopMatrix();
			}

			if(solversCount > 1)
			{
				chain = solversChainRef[characterId][solversCount-2]->outputs[0];
				glColor3f(0,1,0);
				for (int i = 0; i < chain->positions.size(); ++i) 
				{
					Vector3d p = chain->positions[i];
					glPushMatrix();
					glTranslated(p.x(), p.y(), p.z());
					gluSphere(quadric,3,8,8);
					glPopMatrix();
				}
			}
		}
	}
	glEnable(GL_LIGHTING);
}

Quaterniond SolverManager::computeTwist (joint* jt, Vector3d nLook, Vector3d restUp) 
{

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

int intDiv (double n, int d) { return (int) (n / d); }

double mod (double n, int d) {
	double div = n / d;
	int intrem = (int)div * d;
	return n - intrem;
}

void SolverManager::update (int sk, skeleton* s) 
{
	int n = solversChainRef[sk].size();
	if(n == 0) return;
	
	Solver* lastSolver = solversChainRef[sk].back();
	Solver* firstSolver = solversChainRef[sk].front();
	
	if(!lastSolver || !firstSolver)
	{
		printf("[Solver Manager] Los solvers no estan bien inicializados\n");
	}

	// TO_REDO
	// Updating the last solvers gets the right solution... it's not the way to do this.
	// The render will demand the right solution to launch the solver computations.
	lastSolver->update();
	
	Chain* chain = lastSolver->outputs[sk];

	// Move root to initial joint -> correct position?
	Vector3d translation = chain->positions[0] - s->joints[0]->translation;
	s->joints[0]->addTranslation(translation.x(), translation.y(), translation.z());

	// Computing twist correction
	vector<double> jointTwist (chain->positions.size(), 0);
	Quaterniond removedTwist = Quaterniond::Identity();

	clock_t start2 = clock();

	for (int i = 0; i < s->joints.size(); ++i) 
		s->joints[i]->qOrient = s->joints[i]->restOrient;
	
	s->joints[0]->computeWorldPos();

	// First pass to compute twist. TO OPTIMIZE
	Vector3d pp;
	for (int i = 0; i < chain->positions.size()-1; ++i) 
	{
		s->joints[0]->computeWorldPos();
		joint* jt = s->joints[i], * father, * child;

		Vector3d fatherPosition (0,0,0); 
		Quaterniond fatherRotation = Quaterniond::Identity(); 
		Quaterniond fatherTwist = Quaterniond::Identity();

		if(jt->father) 
		{
			father = jt->father;
			fatherPosition = father->translation;
			fatherRotation = father->rotation;
			fatherTwist = father->twist;
		} else jt->translation = fatherPosition + fatherRotation._transformVector(jt->pos);

		if(jt->childs.size()>0)
			child = jt->childs[0];
		else break;

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
		if (i == chain->positions.size()-2 && brains[sk]->lookInit && twistCorrectionEnabled) 
		{
			//Vector3d nLook = solverData->neck._transformVector(s->joints[19]->translation - 
			//													 s->joints[18]->translation).normalized();

			Vector3d nLook = solverData->neck._transformVector(chain->positions[19] - chain->positions[18]).normalized();

			// Twist correcter is the quaternion from u to v, where u and v are projections onto the plane with n = nLook
			// proj(v,n) = v - v.dot(n) * n
			Vector3d quatAxis = nLook;
			Vector3d u = jt->rotation._transformVector(jt->rRotation.inverse()._transformVector(Vector3d(0,1,0))).normalized();
			//if (nLook.x() < 0) u.z() *= -1;

			Vector3d v (0,1,0);
			u = (u - u.dot(quatAxis)*quatAxis).normalized();
			v = (v - v.dot(quatAxis)*quatAxis).normalized();
			Vector3d u2 = jt->rotation.inverse()._transformVector(u);
			Vector3d v2 = jt->rotation.inverse()._transformVector(v);

			Quaterniond twistQuat = Quaterniond::Identity();
			twistQuat.setFromTwoVectors(u, v);

			if (twistCorrectionEnabled) 
			{
				double twistAngle = acos(u.dot(v));
				if (u.isApprox(v,0.001)) twistAngle = 0;
				double dotProd = u.dot(v);
				double s = sign(nLook.x());
				double twistAngleDeg = sign(nLook.x()) * twistAngle * 180 / M_PI;
				if (sign(nLook.x()) < 0) 
				{
					twistAngleDeg *= 1;
				}

				Vector3d u3 = nLook.cross(v);
				Quaterniond positiveQuarter;	positiveQuarter.setFromTwoVectors(u3,v);
				Quaterniond negativeQuarter;	negativeQuarter.setFromTwoVectors(v,u3);

				if (previousLookAngles[sk] > 90) 
				{			// CASO POSITIVO
					if (twistAngleDeg < 0) twistAngleDeg += 360;
					double increment = twistAngleDeg - mod(previousLookAngles[sk], 360);
					previousLookAngles[sk] += increment;

					int numQuarters = intDiv(previousLookAngles[sk], 90);
					Quaterniond extraTurns = Quaterniond::Identity();
					while (numQuarters > 0) 
					{
						extraTurns = extraTurns * negativeQuarter;
						--numQuarters;
					}
					Quaterniond remainder;	remainder.setFromTwoVectors (extraTurns._transformVector(u), v);
					twistQuat = extraTurns * remainder;
				} else if (previousLookAngles[sk] < -90) {	// CASO NEGATIVO
					bool bad;
					bad = true;
				} 
				else {	// CASO NEUTRAL
					if (twistAngleDeg < 0) twistAngleDeg += 360;
					double increment = twistAngleDeg - mod(previousLookAngles[sk], 360);
					previousLookAngles[sk] += increment;
					twistQuat.setFromTwoVectors(u,v);
				}

				Vector3d vvvv = twistQuat.vec();
				twistQuat.vec() = jt->rotation.inverse()._transformVector(vvvv);

				//Vector3d remaining (sin(angleAux*M_PI / 180.0), 0, cos(angleAux*M_PI / 180.0));
				//Quaterniond correction3;	correction3.setFromTwoVectors(Vector3d(0,0,1), remaining);
				//twistQuat = twistQuat * correction1 * correction2 * correction3;
			}

			
			Vector3d currentUp = jt->rotation._transformVector(brains[sk]->restUpVector);
			currentUp = jt->rotation._transformVector(jt->rRotation.inverse()._transformVector(Vector3d(0,1,0)));

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
			s->joints[0]->computeWorldPos();

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
			for (int i = 0; i < smoothingIterations; ++i) {
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


	// Computing look correction
	if (!brains[sk]->lookInit) 
	{
		brains[sk]->lookInit = true;
		//brains[sk]->restUpVector = s->joints[18]->rotation.inverse()._transformVector(Vector3d(0,1,0));
		return;
	}
	
	if (!twistCorrectionEnabled) return;

	// Twist correction application??
	for (int j = numTwisted; j >= 0; --j) {
		int id = chain->positions.size()-2 - j;
		Quaterniond currentTwist = Quaterniond::Identity().slerp(jointTwist[id], removedTwist);
		joint* jt = s->joints[id];

		Quaterniond transformation = Quaterniond::Identity();
		for (int i = chain->positions.size()-2; i > id; --i) {
			joint* jt2 = s->joints[i];
			if (jt2->father != 0) transformation =  transformation * jt2->qOrient.inverse() * jt2->father->qrot.inverse();
		}
			
		Vector3d v = currentTwist.vec();
		Vector3d newV = transformation.inverse()._transformVector(v);
		currentTwist = Quaterniond(currentTwist.w(), newV.x(), newV.y(), newV.z());
		jt->qOrient = jt->qOrient * currentTwist;
	}

	s->joints[0]->computeWorldPos();

}

void SolverManager::newFrame (int frame) 
{
	solverData->time = frame * 1.0 / (solverData->fps);

	// Update dirtyness in solvers that require it
	for (int i = 0; i < solversChainRef.size(); ++i) 
	{
		vector<Solver*>& solverChain = solversChainRef[i];
		for (int j = 0; j < solverChain.size(); ++j) 
		{
			solverChain[j]->updateDirtyness();
			solverChain[j]->time = frame * 1.0 / (solverData->fps);
		}
	}

	// AI actions
	for (int i = 0; i < brains.size(); ++i) 
	{
		brains[i]->think();
	}
}

bool SolverManager::isDirty(int sk) 
{
	return solversChainRef[sk][solversChainRef[sk].size()-1]->dirtyFlag;
}

void SolverManager::addSkeleton(int id, skeleton* s) 
{ 
	solversChainRef[id] = vector<Solver*>();
	brains[id] = new Intelligence(id);
	previousLookAngles[id] = 0;
	numVueltas[id] = 0;
	solverData->skeletons.push_back(s);
}

void SolverManager::addSolver(Solver *s, int sk) 
{
	int lastSolver = solversChainRef[sk].size() - 1;

	if (lastSolver >= 0) {
		if (solversChainRef[sk][lastSolver]->outputs.size() > 1) 
			s->inputs.push_back(solversChainRef[sk][lastSolver]->outputs[sk]);
		else 
			s->inputs.push_back(solversChainRef[sk][lastSolver]->outputs[0]);
		Chain* c = new Chain();	c->positions.resize(20);
		s->outputs.push_back(c);
		solversChainRef[sk][lastSolver]->children.push_back(s);
		s->fathers.push_back(solversChainRef[sk][lastSolver]);
	} else {
		Chain* c = new Chain();	c->positions.resize(20);
		s->outputs.push_back(c);
	}

	solversChainRef[sk].push_back(s);
	s->data = solverData;
}	