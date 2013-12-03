#include "SolverManager.h"
#include "AdriViewer.h"
#include "SolverSinusoidal.h"


SolverManager::SolverManager(void)
{
	solvers = map<int, vector<Solver*> > ();

	verletEnabled = vector<bool>();
	solversEnabled = vector<bool>();
	dividingBaseFactor = 10;

	solverData = new SolverData();
}


SolverManager::~SolverManager(void)
{
}



/*
vector<Eigen::Quaternion<double> > SolverManager::computeSolvers(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk) {

	double fps = 1.0/animationPeriod*1000;
	double currentTime = (double)frame/fps;

	vector<Eigen::Quaternion<double> > finalPositions(skeletons[sk]->joints.size());
	for (int i = 0; i < finalPositions.size(); ++i) finalPositions[i] = Eigen::Quaternion<double>(1,0,0,0);

	if (sk < solversEnabled.size() && !solversEnabled[sk]) return finalPositions;

	for (int i = 0; i < solvers[sk].size(); ++i) {
		Solver* s = solvers[sk][i];
		vector<pair<int, Eigen::Quaternion<double> > > solverPos = s->solve(currentTime);
		for (int j = 0; j < solverPos.size(); ++j) {
			int id = solverPos[j].first;
			Eigen::Quaternion<double> q = solverPos[j].second;
			finalPositions[id] = finalPositions[id] * q;
		}
	}

	return finalPositions;
}*/

/*
vector<Eigen::Quaternion<double> > SolverManager::computePostSolvers(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk) {

	vector<Eigen::Quaternion<double> > finalPositions(skeletons[sk]->joints.size());
	for (int i = 0; i < finalPositions.size(); ++i) finalPositions[i] = Eigen::Quaternion<double>(1,0,0,0);

	for (int i = 0; i < postSolvers[sk].size(); ++i) {
		Solver* s = postSolvers[sk][i];
		vector<pair<int, Eigen::Quaternion<double> > > solverPos = s->solve(frame/24.0);
		for (int j = 0; j < solverPos.size(); ++j) {
			int id = solverPos[j].first;
			Eigen::Quaternion<double> q = solverPos[j].second;
			finalPositions[id] = finalPositions[id] * q;
		}
	}

	return finalPositions;
}*/

/*
vector<Eigen::Quaternion<double> > SolverManager::computeVerlet(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk) {

	vector<Eigen::Quaternion<double> > finalPositions(skeletons[sk]->joints.size());
	for (int i = 0; i < finalPositions.size(); ++i) finalPositions[i] = Eigen::Quaternion<double>(1,0,0,0);

	if (sk >= verletEnabled.size() || !verletEnabled[sk]) return finalPositions;
	SolverVerlet* verlet = verlets[sk];

	// Temp
	glPointSize(8);
	glColor3f(0,1,0);
	glBegin(GL_POINTS);
	for (int i = 0; i < verlet->currentPositions.size(); ++i)
		glVertex3d(verlet->currentPositions[i].x(), verlet->currentPositions[i].y(), verlet->currentPositions[i].z());
	glEnd();

	glColor3f(1,0,0);
	glBegin(GL_LINES);
	for (int i = 0; i < verlet->currentPositions.size()-1; ++i) {
		glVertex3d(verlet->currentPositions[i].x(), verlet->currentPositions[i].y(), verlet->currentPositions[i].z());
		glVertex3d(verlet->currentPositions[i+1].x(), verlet->currentPositions[i+1].y(), verlet->currentPositions[i+1].z());
	}
	glEnd();

	// Draw collisions
	for (int i = 0; i < verlet->currentPositions.size(); ++i) {
		glColor3f(0,1,0);
		Eigen::Vector3d position = verlet->currentPositions[i];
		bool found = false;
		for (int v = 0; v < skeletons.size(); ++v) {
			if (v == sk) continue;
			if (found) break;
			SolverVerlet* sv = verlets[v];
			for (int j = 0; j < sv->currentPositions.size(); ++j) {
				Eigen::Vector3d position2 = sv->currentPositions[j];
				if ((position - position2).norm() < 50) {
					glColor3f(1,0,0);
					found = true;
					break;
				}
			}
		}

		glPushMatrix();
		glTranslated(position.x(), position.y(), position.z());
		GLUquadricObj *quadric;
		quadric = gluNewQuadric();
		gluQuadricDrawStyle(quadric, GLU_LINE );
		gluSphere(quadric,25,8,8);
		glPopMatrix();
	}


	vector<Eigen::Vector3d> lastFramePositions = verlets[sk]->currentPositions;

	verlets[sk]->chain[0].first->computeWorldPos();
	for (int i = 0; i < verlets[sk]->currentPositions.size(); ++i) 
		verlets[sk]->currentPositions[i] = verlets[sk]->chain[i].first->getWorldPosition();

	//verlets[sk]->currentPositions[0] = verlets[sk]->chain[0].first->getWorldPosition();


	// Compute Verlet integration
	double fps = 1.0/animationPeriod*1000;
	double currentTime = (double)frame/fps;
	int numReps = 20;
	for (int k = 0; k < numReps-1; ++k) {
		verlets[sk]->solveVerlet(currentTime + ((double)k / numReps)*animationPeriod/1000.0, verlets, sk);
	}
	vector<pair<int,Eigen::Vector3d> > positions = verlets[sk]->solveVerlet(currentTime + ((double)(numReps-1) / numReps)*animationPeriod/1000.0, verlets, sk);

	// Check if some position has changed
	bool positionsChanged = false;
	for (int i = 0; i < positions.size(); ++i) {
		Eigen::Vector3d lfp = lastFramePositions[i];
		Eigen::Vector3d ver = positions[i].second;
		if ((lastFramePositions[i] - positions[i].second).norm() > 1) {
			positionsChanged = true;
		}
	}

	if (!positionsChanged) {
		for (int i = 1; i < positions.size(); ++i) {
			positions[i].second = lastFramePositions[i];
		}
	}
	
	for (int i = 0; i < positions.size()-1; ++i) {

		int currentID = positions[i].first;

		Eigen::Vector3d currentPos = verlets[sk]->chain[i].first->getWorldPosition();
		Eigen::Vector3d nextPos = verlets[sk]->chain[i+1].first->getWorldPosition();
		Eigen::Vector3d nextVerlet = positions[i+1].second;
		
		glDisable(GL_LIGHTING);
		glColor3f(1,0,0);
		glBegin(GL_LINES);
		glVertex3f(currentPos.x(), currentPos.y(), currentPos.z());
		glVertex3f(nextPos.x(), nextPos.y(), nextPos.z());
		glEnd();
		glColor3f(0,1,0);
		glBegin(GL_LINES);
		glVertex3f(currentPos.x(), currentPos.y(), currentPos.z());
		glVertex3f(nextVerlet.x(), nextVerlet.y(), nextVerlet.z());
		glEnd();
		glEnable(GL_LIGHTING);



		Eigen::Vector3d nextPosE (nextPos.x(), nextPos.y(), nextPos.z());
		Eigen::Vector3d nextVerletE (nextVerlet.x(), nextVerlet.y(), nextVerlet.z());
		Eigen::Vector3d currentPosE (currentPos.x(), currentPos.y(), currentPos.z());

		Quaternion<double> qq = verlets[sk]->chain[i].first->rotation.inverse();
		Eigen::Vector3d v1f = qq._transformVector(nextPosE - currentPosE);
		Eigen::Vector3d v2f = qq._transformVector(nextVerletE - currentPosE);

		v1f.normalize();
		v2f.normalize();

		if (v1f.isApprox(v2f,0.001)) {
			//if (dumpVectors) printf("Vectors equal\n");
			continue;
		}
		
		printf("Node %d:\n", i);
		printf("Current pos: %f %f %f\n", currentPosE.x(), currentPosE.y(), currentPosE.z());
		printf("Next pos: %f %f %f\n", nextPosE.x(), nextPosE.y(), nextPosE.z());
		printf("Verlet pos: %f %f %f\n", nextVerletE.x(), nextVerletE.y(), nextVerletE.z());
		printf("v1: %f %f %f\n", v1f.x(), v1f.y(), v1f.z());
		printf("v2: %f %f %f\n", v2f.x(), v2f.y(), v2f.z());


		Eigen::Quaternion<double> q;
		q.setFromTwoVectors(v1f,v2f);

		bool vectorsTooSimilar = (v1f-v2f).norm() < 0.005;
		bool aVectorIsZero = (v1f.norm() < 0.0005 || v2f.norm() < 0.0005);

		if (vectorsTooSimilar || aVectorIsZero) {
			// Do nothing
		} else {
			verlets[sk]->chain[i].first->addRotation(q);
			verlets[sk]->chain[0].first->computeWorldPos();
		}
	}

	skeletons[sk]->joints[0]->dirtyFlag = true;
	return finalPositions;
}*/