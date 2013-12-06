#pragma once
#include "Solver.h"
#include "SolverVerlet.h"
#include "skeleton.h"
#include "AdriViewer.h"
#include "Chain.h"
#include "Intelligence.h"
#include <vector>
#include <map>

using namespace std;

class SolverManager
{
public:
	SolverData* solverData;

	map<int, vector<Solver*> > solvers;
	map<int, Intelligence*> brains;
	
	vector<bool> verletEnabled;
	vector<bool> solversEnabled;
	
	void newFrame (int frame) {

		solverData->time = frame * 1.0 / (solverData->fps);

		// Update dirtyness in solvers that require it
		for (int i = 0; i < solvers.size(); ++i) {
			vector<Solver*> vsolvers = solvers[i];
			for (int j = 0; j < vsolvers.size(); ++j) {
				vsolvers[j]->updateDirtyness();
				vsolvers[j]->time = frame * 1.0 / (solverData->fps);
			}
		}

		// AI actions
		for (int i = 0; i < brains.size(); ++i) {
			brains[i]->think();
		}
	}

	bool isDirty(int sk) {
		return solvers[sk][solvers[sk].size()-1]->dirtyFlag;
	}

	void draw() {
		GLUquadricObj *quadric;
		quadric = gluNewQuadric();
		gluQuadricDrawStyle(quadric, GLU_LINE );
		glDisable(GL_LIGHTING);
		for (int sk = 0; sk < solvers.size(); ++sk) {
			Solver* lastSolver = solvers[sk][solvers[sk].size()-1];
			Chain* chain = lastSolver->outputs[0];
			glColor3f(1,0,0);
			for (int i = 0; i < chain->positions.size(); ++i) {
				Vector3d p = chain->positions[i];
				glPushMatrix();
				glTranslated(p.x(), p.y(), p.z());
				gluSphere(quadric,2,8,8);
				glPopMatrix();
			}
			chain = solvers[sk][1]->outputs[0];
			glColor3f(0,1,0);
			for (int i = 0; i < chain->positions.size(); ++i) {
				Vector3d p = chain->positions[i];
				glPushMatrix();
				glTranslated(p.x(), p.y(), p.z());
				gluSphere(quadric,2,8,8);
				glPopMatrix();
			}
		}
		glEnable(GL_LIGHTING);
	}

	void update (int sk, skeleton* s) {
		int n = solvers[sk].size();
		Solver* lastSolver = solvers[sk][solvers[sk].size()-1];
		Solver* firstSolver = solvers[sk][0];
		lastSolver->update();
		Chain* chain = lastSolver->outputs[sk];

		// Move root to initial joint
		Vector3d translation = chain->positions[0] - s->joints[0]->translation;
		s->joints[0]->addTranslation(translation.x(), translation.y(), translation.z());

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

			if (i == chain->positions.size()-2) {
				// Twist correction
				//Vector3d nLook = verlets[sk]->lookVector.normalized();
				//Vector3d nLook = s->joints[18]->rotation._transformVector(l->restLookVector).normalized();
				Vector3d nLook = solverData->neck._transformVector(s->joints[19]->translation - s->joints[18]->translation).normalized();
				//Vector3d nLook = (((SolverLook*)solvers[4]))->qrot._transformVector(s->joints[19]->translation - s->joints[18]->translation).normalized();
				Vector3d axis (1,0,0);
				//axis = Vector3d(0,1,0).cross(nLook);
				if (axis.x() < 0) axis *= -1;
				if (nLook.x() < -0.005) {
					axis *= -1;
				}
				double projection = nLook.dot(axis);		// between 0 and 1 since both are normalized

				if (projection > 0) {
					double corrAngle = M_PI/2*projection;
					Vector3d vectorCorrection (0, sin(corrAngle), -cos(corrAngle));
					Vector3d vc1 (0,0,-1);
					Vector3d vc2 = vectorCorrection;
					Quaterniond correction;	correction.setFromTwoVectors(vc1, vc2);
					if (nLook.x() < 0) correction.setFromTwoVectors(vc2, vc1);
					jt->qrot = jt->qrot * correction;
				}
			}
			jt->rotation = fatherRotation * jt->qOrient * jt->qrot;

		}

		s->joints[0]->computeWorldPos();
	}

	void addSkeleton(int id, skeleton* s) { 
		solvers[id] = vector<Solver*>();
		brains[id] = new Intelligence(id);
	}

	void addSolver(Solver *s, int sk) {
		int lastSolver = solvers[sk].size() - 1;

		if (lastSolver >= 0) {
			if (solvers[sk][lastSolver]->outputs.size() > 1) 
				s->inputs.push_back(solvers[sk][lastSolver]->outputs[sk]);
			else 
				s->inputs.push_back(solvers[sk][lastSolver]->outputs[0]);
			Chain* c = new Chain();	c->positions.resize(20);
			s->outputs.push_back(c);
			solvers[sk][lastSolver]->children.push_back(s);
			s->fathers.push_back(solvers[sk][lastSolver]);
		} else {
			Chain* c = new Chain();	c->positions.resize(20);
			s->outputs.push_back(c);
		}

		solvers[sk].push_back(s);
		
	}

	SolverManager(void);
	~SolverManager(void);
};

