#include "BeanViewer.h"

BeanViewer::BeanViewer(QWidget * parent, const QGLWidget * shareWidget,
                   Qt::WindowFlags flags ) : AdriViewer(parent, shareWidget, flags) {
	initScene();
	ReBuildScene();
	solverManager = new SolverManager();
}

void BeanViewer::loadSolvers() {
	SolverSinusoidal* sinY = new SolverSinusoidal(0.1,3,0);	sinY->dimension = 1;
	SolverSinusoidal* sinZ = new SolverSinusoidal(0.2,1,0);	sinZ->dimension = 2;
	SolverSinusoidal* mouthSolver = new SolverSinusoidal(0.3,5,0); mouthSolver->dimension = 2;
	SolverVerlet* verlet = new SolverVerlet();

	int n = escena->skeletons[0]->joints.size();
	escena->skeletons[0]->joints[0]->computeWorldPos();

	// Colocamos la serpiente en (0,0) y en vertical
	Point3d trans = Point3d(0,0,0) - escena->skeletons[0]->joints[0]->pos;
	escena->skeletons[0]->joints[0]->addTranslation(trans.X(), trans.Y(), trans.Z());
	escena->skeletons[0]->joints[0]->addRotation(0,0,90);
	/*escena->skeletons[0]->joints[escena->skeletons[0]->joints.size() - 6]->addRotation(0,0,-90);
	for (int i = 1; i < n-10; ++i) escena->skeletons[0]->joints[i]->addRotation(0,0,-2*(i/8+1));*/
	escena->skeletons[0]->joints[n-6]->addRotation(0,0,-90);
	ReBuildScene();

	// Verlet
	for (int i = 0; i < n-6; ++i) verlet->addJoint(escena->skeletons[0]->joints[i], i);

	// Add joints to sinusoidal solvers
	for (int i = 0; i < n; ++i) {
		if (i > 0 && i < n-5 && i%2 == 0) {
			sinY->addJoint(escena->skeletons[0]->joints[i], i);
			sinZ->addJoint(escena->skeletons[0]->joints[i], i);
		}
		if (i >= n-6 && i % 2 == 0) mouthSolver->addJoint(escena->skeletons[0]->joints[i], i);
	}
		
	verlet->setPositions();
	solverManager->addSolver(sinY);
	solverManager->addSolver(sinZ);
	solverManager->addSolver(mouthSolver);
	solverManager->verlet = verlet;
	solverManager->hasVerlet = true;
	solverManager->oscillation = false;
}

void BeanViewer::draw() {
	// Compute solvers
	if (escena->skeletons.size() > 0 && aniManager.simulationEnabled) {
		vector<Point3d> rots = solverManager->computeSolvers(frame, this->animationPeriod(), escena->skeletons);
		for (int i = 0; i < escena->skeletons[0]->joints.size(); ++i)
			escena->skeletons[0]->joints[i]->addRotation(rots[i].X(), rots[i].Y(), rots[i].Z());
		escena->skeletons[0]->joints[0]->computeWorldPos();
		rots = solverManager->computeVerlet(frame, this->animationPeriod(), escena->skeletons);
		for (int i = 0; i < escena->skeletons[0]->joints.size(); ++i)
			escena->skeletons[0]->joints[i]->addRotation(rots[i].X(), rots[i].Y(), rots[i].Z());
	}
	// Apply skinning
	escena->skinner->computeDeformationsWithSW(escena->skeletons);
	// Call parent function
	AdriViewer::draw();
}