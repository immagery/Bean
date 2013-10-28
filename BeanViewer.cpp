#include "BeanViewer.h"

BeanViewer::BeanViewer(QWidget * parent, const QGLWidget * shareWidget,
                   Qt::WindowFlags flags ) : AdriViewer(parent, shareWidget, flags) {
	initScene();
	ReBuildScene();
	solverManager = new SolverManager();
}

void BeanViewer::loadSolvers() {
	/*SolverSinusoidal* sinY = new SolverSinusoidal(1,3,0);	sinY->dimension = 0;
	SolverSinusoidal* sinZ = new SolverSinusoidal(1,1,0);	sinZ->dimension = 1;
	SolverSinusoidal* mouthSolver = new SolverSinusoidal(0.3,5,0); mouthSolver->dimension = 2;
	SolverStatic* fixed = new SolverStatic();
	SolverVerlet* verlet = new SolverVerlet();
	SolverLook* look = new SolverLook();

	int n = escena->skeletons[0]->joints.size();
	escena->skeletons[0]->joints[0]->computeWorldPos();

	// Colocamos la serpiente en (0,0) y en vertical


	//Eigen::Vector4f lv = escena->skeletons[0]->joints[n-7]->W * (Eigen::Vector4f(lookV.X(), lookV.Y(), lookV.Z(), 1));
	//lookVector = Point3d(lv.x(), lv.y(), lv.z());
	//lookVector = lookVector - p1;

	Point3d trans = Point3d(0,0,0) - escena->skeletons[0]->joints[0]->pos;
	escena->skeletons[0]->joints[0]->addTranslation(trans.X(), trans.Y(), trans.Z());
	
	//escena->skeletons[0]->joints[0]->addRotation(-90,0,0);
	//escena->skeletons[0]->joints[n-6]->addRotation(0,0,-90);
	//for (int i = 1; i < n-10; ++i) escena->skeletons[0]->joints[i]->addRotation(0,0,-2*(i/8+1));

	Eigen::Vector4f invV = escena->skeletons[0]->joints[n-6]->W.inverse() * (Eigen::Vector4f(0, 0, 500, 1));
	Point3d invvv (invV.x(), invV.y(), invV.z());

	Point3d p1 = escena->skeletons[0]->joints[n-6]->getWorldPosition();
	Eigen::Vector4f pe2 = escena->skeletons[0]->joints[n-6]->W * (Eigen::Vector4f(322, -400, 0, 1));
	Eigen::Vector4f pe3 = escena->skeletons[0]->joints[n-6]->W * (Eigen::Vector4f(0, 0, 0, 1));
	Point3d p2(pe2.x(), pe2.y(), pe2.z());
	Point3d p3(pe3.x(), pe3.y(), pe3.z());
	//Point3d p2(0,0,500);
	Point3d lookV = p2 - p3;
	Point3d lookVector = lookV;

	
	
	ReBuildScene();
	escena->skeletons[0]->joints[0]->computeWorldPos();

	// Set looking stuff
	//look->lookPoint = escena->skeletons[0]->joints[n-6]->getWorldPosition() + Point3d(0,1,1000);
	look->lookPoint = escena->skeletons[0]->joints[n-6]->getWorldPosition() + lookVector.normalized()*250;

	//Point3d half = (escena->skeletons[0]->joints[n-5]->getWorldPosition() + escena->skeletons[0]->joints[n-4]->getWorldPosition() ) / 2;
	//Point3d lookVector = escena->skeletons[0]->joints[n-5]->getWorldPosition() - escena->skeletons[0]->joints[n-6]->getWorldPosition();
	//lookVector = Point3d(0,-500,300);
	//Quaternion<double> qt = escena->skeletons[0]->joints[n-7]->qrot;
	//Eigen::Quaterniond qq(qt.W(), qt.X(), qt.Y(), qt.Z());
	//Eigen::Vector3d vv1 = qq._transformVector(Eigen::Vector3d(lookVector.X(), lookVector.Y(), lookVector.Z()));
	//Point3d v1 = Point3d(vv1.x(), vv1.y(), vv1.z()).normalized();

	look->lookVector = lookVector;
	//look->lookVector = v1;

	look->addJoint(escena->skeletons[0]->joints[n-6], n-6);

	// Verlet
	for (int i = 0; i < n-10; ++i) verlet->addJoint(escena->skeletons[0]->joints[i], i);

	// Add joints to sinusoidal solvers
	for (int i = 0; i < n; ++i) {
		if (i >= 0 && i < n-5 && i%2 == 0) {
			sinY->addJoint(escena->skeletons[0]->joints[i], i);
			sinZ->addJoint(escena->skeletons[0]->joints[i], i);
		}
		if (i >= n-6 && i % 2 == 0) mouthSolver->addJoint(escena->skeletons[0]->joints[i], i);
	}
		
	verlet->setPositions();
	//solverManager->addSolver(sinY);
	//solverManager->addSolver(sinZ);
	//solverManager->addSolver(mouthSolver);
	solverManager->addPostSolver(look);
	solverManager->verlet = verlet;
	solverManager->hasVerlet = true;
	solverManager->oscillation = false;*/

	SolverSinusoidal* sin0 = new SolverSinusoidal(2,1,0);	sin0->dimension = 1;
	SolverSinusoidal* sin1 = new SolverSinusoidal(1,2,2);	sin1->dimension = 0;
	SolverVerlet* verlet = new SolverVerlet();
	SolverVerlet* verlet2 = new SolverVerlet();

	// Add skeletons to solver manager
	solverManager->addSkeleton(0);
	solverManager->addSkeleton(1);
	solverManager->addSkeleton(2);
	solverManager->addSkeleton(3);
	


	// Add joints to sinusoidal solvers
	for (int sk = 0; sk < escena->skeletons.size(); ++sk) {
		skeleton* s = escena->skeletons[sk];
		int n = s->joints.size();

		//s->joints[0]->addRotation(-90,0,0);
		//s->joints[n-6]->addRotation(0,0,-90);
		//for (int i = 1; i < n-10; ++i) escena->skeletons[0]->joints[i]->addRotation(0,0,-2*(i/8+1));

		for (int i = 0; i < n; ++i) {
			if (i >= 0 && i < n-5 && i%2 == 0) {
				if (sk % 2 == 0) sin0->addJoint(s->joints[i], i);
				if (sk % 2 == 1) sin1->addJoint(s->joints[i], i);
			}
		}

		// Add joints to verlet solver
		for (int i = 0; i < n-10; ++i) {
			if (sk == 0) verlet->addJoint(s->joints[i], i);
			if (sk == 1) verlet2->addJoint(s->joints[i], i);
		}

		s->joints[0]->computeWorldPos();
	}

	sin0->setPositions();
	sin1->setPositions();
	verlet->setPositions();
	verlet2->setPositions();
	solverManager->verlets[0] = verlet;
	solverManager->verlets[1] = verlet2;
	solverManager->addSolver(sin0,0);
	solverManager->addSolver(sin1,1);
	solverManager->addSolver(sin0,2);
	solverManager->addSolver(sin1,3);
	
}

void BeanViewer::draw() {

	if (frame > 20) {

	}

	if (escena->skeletons.size() > 0 && aniManager.simulationEnabled) {

		for (int sk = 0; sk < escena->skeletons.size(); ++sk) {
			skeleton* s = escena->skeletons[sk];
			s->joints[0]->computeWorldPos();
			vector<Quaternion<double> > rots = solverManager->computeSolvers(frame * 10, this->animationPeriod(), escena->skeletons, sk);
			Quaternion<double> orientInverse = s->joints[0]->qOrient.Inverse();
			for (int i = 0; i < s->joints.size(); ++i) {
				if (rots[i] != Quaternion<double>(1,0,0,0))
					s->joints[i]->addRotation(orientInverse * rots[i] * orientInverse.Inverse());
			}

			Point3d v1 = s->joints[2]->getWorldPosition() - s->joints[1]->getWorldPosition();
			s->joints[1]->pos;
			Point3d v2 (0,100,0);
			vcg::Quaternion<double> q;
			q.FromAxis(acos(v1.dot(v2) / (v1.Norm() * v2.Norm())), v1^v2);
			q.Normalize();
			s->joints[1]->addRotation(orientInverse * q * orientInverse.Inverse());
			s->joints[0]->dirtyFlag = true;
			

			//solverManager->computeVerlet(frame, this->animationPeriod(), escena->skeletons, sk);

		}

		
		
		//for (int i = 0; i < escena->skeletons[0]->joints.size(); ++i)
		//	escena->skeletons[0]->joints[i]->addRotation(rots[i].X(), rots[i].Y(), rots[i].Z());
		//escena->skeletons[0]->joints[0]->computeWorldPos();
		
		/*rots = solverManager->computePostSolvers(frame, this->animationPeriod(), escena->skeletons);
		for (int i = 0; i < escena->skeletons[0]->joints.size(); ++i) {
			if (rots[i] != Quaternion<double>(1,0,0,0)) {
				escena->skeletons[0]->joints[i]->addRotation(rots[i]);
				escena->skeletons[0]->joints[0]->dirtyFlag = true;
			}
		}*/

		//escena->skeletons[0]->joints[0]->computeWorldPos();
	}
	// Call parent function
	AdriViewer::draw();
}