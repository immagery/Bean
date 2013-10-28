#include "SolverManager.h"
#include "AdriViewer.h"


SolverManager::SolverManager(void)
{
	solvers = map<int, vector<Solver*> > ();
	postSolvers = map<int, vector<Solver*> > ();
	verlets = vector<SolverVerlet*> ();

	verletEnabled = vector<bool>();
	solversEnabled = vector<bool>();
	dividingBaseFactor = 10;
}


SolverManager::~SolverManager(void)
{
}

vector<Quaternion<double> > SolverManager::computeSolvers(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk) {

	vector<Quaternion<double> > finalPositions(skeletons[sk]->joints.size());
	for (int i = 0; i < finalPositions.size(); ++i) finalPositions[i] = Quaternion<double>(1,0,0,0);

	if (sk < solversEnabled.size() && !solversEnabled[sk]) return finalPositions;

	for (int i = 0; i < solvers[sk].size(); ++i) {
		Solver* s = solvers[sk][i];
		vector<pair<int, Quaternion<double> > > solverPos = s->solve(frame/24.0);
		for (int j = 0; j < solverPos.size(); ++j) {
			int id = solverPos[j].first;
			finalPositions[id] = solverPos[j].second * finalPositions[id];
		}
	}

	return finalPositions;
}

vector<Quaternion<double> > SolverManager::computePostSolvers(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk) {

	vector<Quaternion<double> > finalPositions(skeletons[sk]->joints.size());
	for (int i = 0; i < finalPositions.size(); ++i) finalPositions[i] = Quaternion<double>(1,0,0,0);

	for (int i = 0; i < postSolvers[sk].size(); ++i) {
		Solver* s = postSolvers[sk][i];
		vector<pair<int, Quaternion<double> > > solverPos = s->solve(frame/24.0);
		for (int j = 0; j < solverPos.size(); ++j) {
			int id = solverPos[j].first;
			finalPositions[id] = solverPos[j].second * finalPositions[id];
		}
	}

	return finalPositions;
}


vector<Quaternion<double> > SolverManager::computeVerlet(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk) {

	vector<Quaternion<double> > finalPositions(skeletons[sk]->joints.size());
	for (int i = 0; i < finalPositions.size(); ++i) finalPositions[i] = Quaternion<double>(1,0,0,0);

	if (sk >= verletEnabled.size() || !verletEnabled[sk]) return finalPositions;
	SolverVerlet* verlet = verlets[sk];

	// Temp
	glPointSize(8);
	glColor3f(0,1,0);
	glBegin(GL_POINTS);
	for (int i = 0; i < verlet->currentPositions.size(); ++i)
		glVertex3d(verlet->currentPositions[i].X(), verlet->currentPositions[i].Y(), verlet->currentPositions[i].Z());
	glEnd();

	// Draw collisions
	for (int i = 0; i < verlet->currentPositions.size(); ++i) {
		glColor3f(0,1,0);
		Point3d position = verlet->currentPositions[i];
		bool found = false;
		for (int v = 0; v < skeletons.size(); ++v) {
			if (v == sk) continue;
			if (found) break;
			SolverVerlet* sv = verlets[v];
			for (int j = 0; j < sv->currentPositions.size(); ++j) {
				Point3d position2 = sv->currentPositions[j];
				if ((position - position2).Norm() < 30) {
					glColor3f(1,0,0);
					found = true;
					break;
				}
			}
		}

		glPushMatrix();
		glTranslated(position.X(), position.Y(), position.Z());
		GLUquadricObj *quadric;
		quadric = gluNewQuadric();
		gluQuadricDrawStyle(quadric, GLU_LINE );
		gluSphere(quadric,15,8,8);
		glPopMatrix();
	}

	glColor3f(1,0,0);
	glBegin(GL_LINES);
	for (int i = 0; i < verlet->currentPositions.size()-1; ++i) {
		glVertex3d(verlet->currentPositions[i].X(), verlet->currentPositions[i].Y(), verlet->currentPositions[i].Z());
		glVertex3d(verlet->currentPositions[i+1].X(), verlet->currentPositions[i+1].Y(), verlet->currentPositions[i+1].Z());
	}
	glEnd();


	vector<Point3d> lastFramePositions = verlets[sk]->currentPositions;

	verlets[sk]->chain[0].first->computeWorldPos();
	verlets[sk]->currentPositions[0] = verlets[sk]->chain[0].first->getWorldPosition();

	/*for (int i = 0; i < lastFramePositions.size(); ++i) {
		Point3d lp = verlets[sk]->chain[i].first->getWorldPosition();
		Point3d lfp = lastFramePositions[i];
		lastFramePositions[i] = lp;
	}*/


	// Compute Verlet integration
	double fps = 1.0/animationPeriod*1000;
	double currentTime = (double)frame/fps;
	int numReps = 20;
	for (int k = 0; k < numReps-1; ++k) {
		verlets[sk]->solveVerlet(currentTime + ((double)k / numReps)*animationPeriod/1000.0, verlets, sk);
	}
	vector<pair<int,Point3d> > positions = verlets[sk]->solveVerlet(currentTime + ((double)(numReps-1) / numReps)*animationPeriod/1000.0, verlets, sk);

	// Check if some position has changed
	bool positionsChanged = false;
	for (int i = 0; i < positions.size(); ++i) {
		if ((lastFramePositions[positions[i].first] - positions[i].second).Norm() > 0.5) positionsChanged = true;
	}

	
	Quaternion<double> orientInverse = skeletons[0]->joints[0]->qOrient.Inverse();

	for (int i = 0; i < positions.size()-1; ++i) {

		int currentID = positions[i].first;

		Point3d worldDiff = verlets[sk]->chain[i+1].first->getWorldPosition() - verlets[sk]->chain[i].first->getWorldPosition();
		Point3d v1 = (lastFramePositions[i+1] - lastFramePositions[i]);
		v1 = worldDiff;
		Point3d v2 = positions[i+1].second - positions[i].second;
		
		
		if (!positionsChanged) {
			Point3d currentWorld = verlets[sk]->chain[positions[i].first].first->getWorldPosition();
			Point3d nextWorld = verlets[sk]->chain[positions[i+1].first].first->getWorldPosition();

			// Using lastFramePositions?
			//currentWorld = positions[i].second;
			//nextWorld = lastFramePositions[i+1];

			Point3d currentVerlet = positions[i].second;
			Point3d nextVerlet = positions[i+1].second;
			bool b = (currentWorld - nextWorld - currentVerlet - nextVerlet) == Point3d(0,0,0);
			v1 = nextWorld - currentWorld;
			v2 = nextVerlet - currentWorld;
			if (b) b = false;
		} 

		bool vectorsTooSimilar = (v1-v2).Norm() < 0.005;
		bool aVectorIsZero = (v1.Norm() < 0.0005 || v2.Norm() < 0.0005);
		bool rotationTooHigh = acos(v1.dot(v2) / (v1.Norm() * v2.Norm())) * 180 / M_PI > 90;

		vcg::Quaternion<double> q;
		q.FromAxis(acos(v1.dot(v2) / (v1.Norm() * v2.Norm())), v1^v2);
		q.Normalize();	q = orientInverse * q * orientInverse.Inverse();
		double rx, ry, rz;	q.ToEulerAngles(rx,ry,rz);
		rx = (rx *360)/(M_PI*2);
		ry = (ry *360)/(M_PI*2);
		rz = (rz *360)/(M_PI*2);

		double dividingFactor = dividingBaseFactor+i;
		if (positionsChanged) {
			rx /= dividingFactor;
			ry /= dividingFactor;
			rz /= dividingFactor;
			q.FromEulerAngles(rx,ry,rz);
			q.HomoNormalize();
			//q /= dividingFactor;
			//q.Normalize();
		}
		if ((vectorsTooSimilar || aVectorIsZero || rotationTooHigh)) {
			//verlets[sk]->chain[positions[i].first].first->addRotation(0,0,0);
		}
		else {
			//verlets[sk]->chain[positions[i].first].first->addRotation(orientInverse * q * orientInverse.Inverse());
			//verlets[sk]->chain[positions[i].first].first->addRotation(q);
			verlets[sk]->chain[positions[i].first].first->addRotation(rx,ry,rz);
			verlets[sk]->chain[0].first->computeWorldPos();
		}
	}

	Point3d v2 = verlets[sk]->chain[positions[positions.size()-1].first].first->getWorldPosition() -
							verlets[sk]->chain[positions[positions.size()-2].first].first->getWorldPosition();		// desired
	Point3d v1 = skeletons[sk]->joints[positions[positions.size()-1].first+1]->getWorldPosition() -
						verlets[sk]->chain[positions[positions.size()-1].first].first->getWorldPosition();
	v1.Normalize();	v2.Normalize();
	vcg::Quaternion<double> q;
	q.FromAxis(acos(v1.dot(v2) / (v1.Norm() * v2.Norm())), v1^v2);
	q.Normalize();
	double rx, ry, rz;	q.ToEulerAngles(rx,ry,rz);
	rx = (rx *360)/(M_PI*2);	rx /= 10;
	ry = (ry *360)/(M_PI*2);	ry /= 10;
	rz = (rz *360)/(M_PI*2);	rz /= 10;
	//verlets[sk]->chain[positions[positions.size()-1].first].first->addRotation(rx,ry,rz);

	skeletons[sk]->joints[0]->dirtyFlag = true;
	return finalPositions;
}