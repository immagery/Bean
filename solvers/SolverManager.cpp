#include "SolverManager.h"
#include "AdriViewer.h"
#include "SolverSinusoidal.h"


SolverManager::SolverManager(void)
{
	solvers = map<int, vector<Solver*> > ();
	postSolvers = map<int, vector<Solver*> > ();
	verlets = vector<SolverVerlet*> ();

	verletEnabled = vector<bool>();
	solversEnabled = vector<bool>();
	dividingBaseFactor = 10;

	dumpVectors = false;
}


SolverManager::~SolverManager(void)
{
}

vector<Eigen::Quaternion<double> > SolverManager::solve(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk) {
	vector<Eigen::Quaterniond> result(skeletons[sk]->joints.size(), Quaterniond(1,0,0,0));
	vector<bool> updated (skeletons[sk]->joints.size(), false);


	skeleton* s = skeletons[sk];

	double fps = 1.0/animationPeriod*1000;
	double currentTime = (double)frame/fps;

	// READ CURRENT STATE
	Quaterniond neckQuaternion = s->joints[19]->rotation;
	verlets[sk]->lookVector = neckQuaternion._transformVector(verlets[sk]->lookVectorResting);
	// END OF READ CURRENT STATE


	/*vector<Solver*> preSolvers = solvers[sk];
	for (int i = 0; i < preSolvers.size(); ++i) {
		vector<pair<int,Eigen::Vector3d > > positions = preSolvers[i]->solve(currentTime);
		for (int j = 0; j < positions.size()-1; ++j) {
			idealChains[sk]->positions[positions[j].first] += positions[j].second;
			if (j == positions.size()-2) idealChains[sk]->positions[positions[j+1].first] += positions[j].second;
		}
	}*/

	// Draw stuff
	for (int i = 0; i <  currentChains[sk]->positions.size(); ++i) {
		glColor3f(0,1,0);
		Eigen::Vector3d position = idealChains[sk]->positions[i];
		glPushMatrix();
		glTranslated(position.x(), position.y(), position.z());
		GLUquadricObj *quadric;
		quadric = gluNewQuadric();
		gluQuadricDrawStyle(quadric, GLU_LINE );
		gluSphere(quadric,2,8,8);
		glPopMatrix();

		glColor3f(1,0,0);
		position = currentChains[sk]->positions[i];
		glPushMatrix();
		glTranslated(position.x(), position.y(), position.z());
		quadric = gluNewQuadric();
		gluQuadricDrawStyle(quadric, GLU_LINE );
		gluSphere(quadric,2,8,8);
		glPopMatrix();
	}

	glColor3f(1,0,1);
	glPushMatrix();
	glTranslated(verlets[sk]->lookPoint.x(), verlets[sk]->lookPoint.y(), verlets[sk]->lookPoint.z());
	GLUquadricObj *quadric;
	quadric = gluNewQuadric();
	gluQuadricDrawStyle(quadric, GLU_LINE );
	gluSphere(quadric,3,8,8);
	glPopMatrix();

	int n = currentChains[sk]->positions.size();
	glColor3f(1,1,0);
	glBegin(GL_LINES);
	Vector3d p = currentChains[sk]->positions[n-2];
	p = p + verlets[sk]->lookVector*3;
	glVertex3d(currentChains[sk]->positions[n-2].x(), currentChains[sk]->positions[n-2].y(), currentChains[sk]->positions[n-2].z());
	glVertex3d(p.x(), p.y(), p.z());
	glEnd();
	glColor3f(1,0,1);
	glBegin(GL_LINES);
	glVertex3d(currentChains[sk]->positions[n-2].x(), currentChains[sk]->positions[n-2].y(), currentChains[sk]->positions[n-2].z());
	glVertex3d(verlets[sk]->lookPoint.x(), verlets[sk]->lookPoint.y(), verlets[sk]->lookPoint.z());
	glEnd();


	// End of drawing

	// Set verlet's ideal positions
	for (int i = 0; i < verlets[sk]->idealPositions.size(); ++i)
		verlets[sk]->idealPositions[i] = idealChains[sk]->positions[i];

	// Verlet integration
	for (int i = 0; i < verlets[sk]->currentPositions.size(); ++i)
		verlets[sk]->currentPositions[i] = verlets[sk]->lastPositions[i] = currentChains[sk]->positions[i];

	int numReps = 10;
	vector<pair<int,Eigen::Vector3d > > positions; 
	for (int k = 0; k < numReps; ++k)
		positions = verlets[sk]->solveVerlet(currentTime + ((double)k / numReps)*animationPeriod/1000.0, verlets, sk);
	for (int j = 0; j < positions.size(); ++j) {
		currentChains[sk]->positions[positions[j].first] = positions[j].second;
		updated[positions[j].first] = true;
	}
	// end of verlet integration

	// Move skeleton to match the chain
	Vector3d translation = currentChains[sk]->positions[0] - skeletons[sk]->joints[0]->getWorldPosition();
	skeletons[sk]->joints[0]->addTranslation(translation.x(), translation.y(), translation.z());


	//for (int i = 0; i < currentChains[sk]->positions.size()-1; ++i) {
	for (int i = 1; i < currentChains[sk]->positions.size(); ++i) {
		skeletons[sk]->joints[0]->computeWorldPos();

		joint* father = s->joints[i]->father;

		Vector3d fatherPosition = father->getWorldPosition();
		Vector3d currentVector = s->joints[i]->getWorldPosition() - fatherPosition;
		Vector3d desiredVector = currentChains[sk]->positions[i] - fatherPosition;

		printf("Joint %d:\n", i);

		Quaterniond fatherRotInverse = father->rotation.inverse();
		Quaterniond fatherRot = father->rotation;

		printf("v1: %f %f %f\n", currentVector.x(), currentVector.y(), currentVector.z());
		printf("v2: %f %f %f\n", desiredVector.x(), desiredVector.y(), desiredVector.z());

		printf("	Initial father qrot: %f %f %f %f\n", father->qrot.x(), father->qrot.y(), father->qrot.z(), father->qrot.w());

		// --- Method 1
		Eigen::Vector3d v1 = fatherRotInverse._transformVector(currentVector);
		Eigen::Vector3d v2 = fatherRotInverse._transformVector(desiredVector);
		Vector3d v12 = father->qrot.inverse()._transformVector(currentVector);
		Quaternion<double> q;	q.setFromTwoVectors(v1,v2);

		printf("	Deltaq: %f %f %f %f\n", q.x(), q.y(), q.z(), q.w());

		Vector3d f1 = father->qrot._transformVector(currentVector);

		father->addRotation(q);

		Vector3d f2 = father->qrot._transformVector(currentVector);

		printf("	Father qrot 1: %f %f %f %f\n", father->qrot.x(), father->qrot.y(), father->qrot.z(), father->qrot.w());

		// --- Method 2
		Vector3d w = father->rRotation._transformVector(s->joints[i]->pos);
		w = father->rotation.inverse()._transformVector(w);
		q.setFromTwoVectors(w, v2);

		s->joints[0]->computeWorldPos();

		printf("	Father qrot 2: %f %f %f %f\n\n", q.x(), q.y(), q.z(), q.w());

		Vector3d r = father->rotation._transformVector(s->joints[i]->pos);
		printf("	Result vector: %f %f %f\n", r.x(), r.y(), r.z());
	}

	s->joints[0]->computeWorldPos();

	return result;
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