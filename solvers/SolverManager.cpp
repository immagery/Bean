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

	dumpVectors = false;
}


SolverManager::~SolverManager(void)
{
}

vector<vcg::Quaternion<double> > SolverManager::computeSolvers(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk) {

	vector<vcg::Quaternion<double> > finalPositions(skeletons[sk]->joints.size());
	for (int i = 0; i < finalPositions.size(); ++i) finalPositions[i] = vcg::Quaternion<double>(1,0,0,0);

	if (sk < solversEnabled.size() && !solversEnabled[sk]) return finalPositions;

	for (int i = 0; i < solvers[sk].size(); ++i) {
		Solver* s = solvers[sk][i];
		vector<pair<int, Eigen::Quaternion<double> > > solverPos = s->solve(frame/24.0);
		for (int j = 0; j < solverPos.size(); ++j) {
			int id = solverPos[j].first;
			vcg::Quaternion<double> q(solverPos[j].second.w(),
								solverPos[j].second.x(),
								solverPos[j].second.y(),
								solverPos[j].second.z());
			finalPositions[id] = q * finalPositions[id];
		}
	}

	return finalPositions;
}

vector<vcg::Quaternion<double> > SolverManager::computePostSolvers(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk) {

	vector<vcg::Quaternion<double> > finalPositions(skeletons[sk]->joints.size());
	for (int i = 0; i < finalPositions.size(); ++i) finalPositions[i] = vcg::Quaternion<double>(1,0,0,0);

	for (int i = 0; i < postSolvers[sk].size(); ++i) {
		Solver* s = postSolvers[sk][i];
		vector<pair<int, Eigen::Quaternion<double> > > solverPos = s->solve(frame/24.0);
		for (int j = 0; j < solverPos.size(); ++j) {
			int id = solverPos[j].first;
			vcg::Quaternion<double> q(solverPos[j].second.w(),
								solverPos[j].second.x(),
								solverPos[j].second.y(),
								solverPos[j].second.z());
			finalPositions[id] = q * finalPositions[id];
		}
	}

	return finalPositions;
}


vector<vcg::Quaternion<double> > SolverManager::computeVerlet(int frame, int animationPeriod, const vector<skeleton*>& skeletons, int sk) {

	vector<vcg::Quaternion<double> > finalPositions(skeletons[sk]->joints.size());
	for (int i = 0; i < finalPositions.size(); ++i) finalPositions[i] = vcg::Quaternion<double>(1,0,0,0);

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
		gluSphere(quadric,25,8,8);
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

	if (verlets[sk]->chain[0].first->getWorldPosition().Y() > 1) {
		bool b = false;
	}

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
		Point3d lfp = lastFramePositions[i];
		Point3d ver = positions[i].second;
		if ((lastFramePositions[i] - positions[i].second).Norm() > 1) {
			positionsChanged = true;
		}
	}

	if (!positionsChanged) {
		for (int i = 1; i < positions.size(); ++i) {
			positions[i].second = lastFramePositions[i];
		}
	}

	
	bool exitDump = false;

	for (int i = 0; i < positions.size()-1; ++i) {

		int currentID = positions[i].first;

		Point3d worldDiff = verlets[sk]->chain[i+1].first->getWorldPosition() - verlets[sk]->chain[i].first->getWorldPosition();
		//worldDiff = verlets[sk]->chain[i+1].first->getWorldPosition() - positions[i].second;
		Point3d v1 = (lastFramePositions[i+1] - positions[i].second);
		v1 = worldDiff;
		Point3d v2 = positions[i+1].second - positions[i].second;
		v2 = positions[i+1].second - verlets[sk]->chain[i].first->getWorldPosition();

		Point3d currentPos = verlets[sk]->chain[i].first->getWorldPosition();
		Point3d nextPos = verlets[sk]->chain[i+1].first->getWorldPosition();
		Point3d nextVerlet = positions[i+1].second;

		v1 = nextPos - currentPos;
		v2 = nextVerlet - currentPos;
		
		glDisable(GL_LIGHTING);
		glColor3f(1,0,0);
		glBegin(GL_LINES);
		glVertex3f(currentPos.X(), currentPos.Y(), currentPos.Z());
		glVertex3f(nextPos.X(), nextPos.Y(), nextPos.Z());
		glEnd();
		glColor3f(0,1,0);
		glBegin(GL_LINES);
		glVertex3f(currentPos.X(), currentPos.Y(), currentPos.Z());
		glVertex3f(nextVerlet.X(), nextVerlet.Y(), nextVerlet.Z());
		glEnd();
		glEnable(GL_LIGHTING);


		Point3d v1f, v2f;
		
		
		/*if (!positionsChanged) {
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
		} */

		v1.Normalize();
		v2.Normalize();

		Eigen::Vector4f nextPosE (nextPos.X(), nextPos.Y(), nextPos.Z(), 1);
		Eigen::Vector4f nextVerletE (nextVerlet.X(), nextVerlet.Y(), nextVerlet.Z(), 1);
		Eigen::Vector4f currentPosE (currentPos.X(), currentPos.Y(), currentPos.Z(), 1);
		Eigen::Matrix4f W = verlets[sk]->chain[positions[i].first].first->W.transpose();
		Eigen::Vector4f v11 = W * nextPosE - W * currentPosE;
		Eigen::Vector4f v12 = W * nextVerletE - W * currentPosE;


		//Eigen::Quaternion<double> qq;
		//qq.setFromTwoVectors(v11, v12);

		vcg::Quaternion<double> q;
		//Eigen::Vector4f v11 = verlets[sk]->chain[positions[i].first].first->world.inverse() * Eigen::Vector4f(v1.X(), v1.Y(), v1.Z(), 1);
		//Eigen::Vector4f v12 = verlets[sk]->chain[positions[i].first].first->world.inverse() * Eigen::Vector4f(v2.X(), v2.Y(), v2.Z(), 1);
		v1f = Point3d(v11.x(), v11.y(), v11.z());
		v2f = Point3d(v12.x(), v12.y(), v12.z());

		v1f.Normalize();
		v2f.Normalize();

		bool vectorsTooSimilar = (v1f-v2f).Norm() < 0.005;
		bool aVectorIsZero = (v1f.Norm() < 0.0005 || v2f.Norm() < 0.0005);
		bool rotationTooHigh = acos(v1f.dot(v2f) / (v1f.Norm() * v2f.Norm())) * 180 / M_PI > 90;
		rotationTooHigh = false;

		//v1f.Normalize();
		//v2f.Normalize();
		q.FromAxis(acos(v1f.dot(v2f) / (v1f.Norm() * v2f.Norm())), v1f^v2f);
		//Eigen::Vector4f v = verlets[sk]->chain[positions[i].first].first->W.inverse() * Eigen::Vector4f(q.X(), q.Y(), q.Z(), 1);
		//Eigen::Vector4f v0 = verlets[sk]->chain[positions[i].first].first->W.inverse() * Eigen::Vector4f(0,0,0,1);
		/*v0 = v - v0;
		q.X() = v0.x();
		q.Y() = v0.y();
		q.Z() = v0.z();*/
		q.Normalize();

		/*if (dumpVectors || !dumpVectors) {
			if ( i <= 10) {
				if ((vectorsTooSimilar || aVectorIsZero || rotationTooHigh)) {
					if (vectorsTooSimilar) printf("Vectors are too similar\n");
					if (aVectorIsZero) printf("A vector is zero\n");
					if (rotationTooHigh) printf("Rotation too high\n");
				} else {
					printf("Node %d:\n", i);
					printf("Verlet position: %f %f %f\n", positions[i].second.X(), positions[i].second.Y(), positions[i].second.Z());
					printf("Old vector (v1): %f %f %f\n", v1.X(), v1.Y(), v1.Z());
					printf("Next vector (v2): %f %f %f\n", v2.X(), v2.Y(), v2.Z());
					printf("Old vector (v1f): %f %f %f\n", v1f.X(), v1f.Y(), v1f.Z());
					printf("Next vector (v2f): %f %f %f\n", v2f.X(), v2f.Y(), v2f.Z());
					Point3d axis = (v1f^v2f).normalized();
					printf("Rot. axis: %f %f %f\n", axis.X(), axis.Y(), axis.Z());
					printf("Rot. amount: %f\n", q.W());
					exitDump = true;
				}
			}
		}*/

		double rx, ry, rz;	q.ToEulerAngles(rx,ry,rz);
		rx = (rx *360)/(M_PI*2);
		ry = (ry *360)/(M_PI*2);
		rz = (rz *360)/(M_PI*2);

		double dividingFactor = dividingBaseFactor+i;
		//rx /= 2;	ry /= 2;	rz /= 2;
		/*if (positionsChanged) {
			rx /= dividingFactor;
			ry /= dividingFactor;
			rz /= dividingFactor;
			q.FromEulerAngles(rx,ry,rz);
			q.HomoNormalize();
			//q /= dividingFactor;
			//q.Normalize();
		}*/
		if ((vectorsTooSimilar || aVectorIsZero || rotationTooHigh)) {
			//verlets[sk]->chain[positions[i].first].first->addRotation(0,0,0);
			if (vectorsTooSimilar) printf("Vectors are too similar\n");
			if (aVectorIsZero) printf("A vector is zero\n");
			if (rotationTooHigh) printf("Rotation too high\n");
		}
		else {
			//verlets[sk]->chain[positions[i].first].first->addRotation(orientInverse * q * orientInverse.Inverse());
			//verlets[sk]->chain[positions[i].first].first->addRotation(q);
			//Point3d rot (rx, ry, rz);
			Eigen::Vector4f rot(rx, ry, rz, 1);
			//rot = verlets[sk]->chain[positions[i].first].first->W * rot;
			//rot = verlets[sk]->chain[positions[i].first].first->world.inverse() * rot;
			//Matrix33d mat;	verlets[sk]->chain[0].first->qOrient.ToMatrix(mat);
			//rot = mat * rot;
			//printf("Node %d rot: %f %f %f\n", i, rot.x(), rot.y(), rot.z());
			verlets[sk]->chain[positions[i].first].first->addRotation(rot.x(), rot.y(), rot.z());
			//verlets[sk]->chain[positions[i].first].first->addRotation(q);

			verlets[sk]->chain[0].first->computeWorldPos();
		}
	}

	if (exitDump) dumpVectors = false;
	exitDump = false;

	/*Point3d v2 = verlets[sk]->chain[positions[positions.size()-1].first].first->getWorldPosition() -
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
	rz = (rz *360)/(M_PI*2);	rz /= 10;*/
	//verlets[sk]->chain[positions[positions.size()-1].first].first->addRotation(rx,ry,rz);

	skeletons[sk]->joints[0]->dirtyFlag = true;
	return finalPositions;
}