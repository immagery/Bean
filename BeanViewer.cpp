#include "BeanViewer.h"
#include "util.h"

BeanViewer::BeanViewer(QWidget * parent, const QGLWidget * shareWidget,
                   Qt::WindowFlags flags ) : AdriViewer(parent, shareWidget, flags) {
	initScene();
	ReBuildScene();
	solverManager = new SolverManager();
	particles = new Particles();
}

void BeanViewer::loadSolvers() {
	
	for (int sk = 0; sk < escena->skeletons.size(); ++sk) {
		skeleton* s = escena->skeletons[sk];
		
		int n = s->joints.size();

		SolverSinusoidal* sin = new SolverSinusoidal(15,2 + rand()%5,rand()%10);	sin->dimension = 0;
		SolverSinusoidal* sin2 = new SolverSinusoidal(15,2 + rand()%5,rand()%10);	sin2->dimension = 2;
		SolverVerlet* verlet = new SolverVerlet();

		Vector3d translation = Vector3d((sk % 3) * 75, 0, -200 + (sk / 3)*100) - s->joints[0]->getWorldPosition();
		s->joints[0]->addTranslation(translation.x(), translation.y(), translation.z());
		s->joints[0]->addRotation(0,0,90);
		s->joints[n-6]->addRotation(0,0,-90);
		//s->joints[0]->addRotation(0,0,-30);
		//for (int i = 1; i < n-15; ++i) s->joints[i]->addRotation(0,0,-2*(i/2));

		s->joints[0]->computeWorldPos();

		solverManager->addSkeleton(sk, s);

		// Add joints to sinusoidal solvers
		for (int i = 1; i < solverManager->currentChains[sk]->positions.size(); ++i) {
			//if (i >= 0 && i < n-8 && i%2 == 0) {
			//if (i < n-10) {
				solverManager->addJointToSolver(sk, i, sin);
				solverManager->addJointToSolver(sk, i, sin2);
			//}
		}

		// Add joints to verlet solver
		for (int i = 0; i < solverManager->currentChains[sk]->positions.size(); ++i)
			solverManager->addJointToSolver(sk, i, verlet);

		// Add joints to look solver
		int nn = solverManager->currentChains[sk]->positions.size();
		Vector3d increment = solverManager->currentChains[sk]->positions[nn-1] - solverManager->currentChains[sk]->positions[nn-2];
		verlet->lookPoint = solverManager->currentChains[sk]->positions[nn-2] + (increment*5) + Vector3d(0,-100,0);
		//verlet->lookPoint += Vector3d(0,-60,200);
		//verlet->lookOffset = 
		//verlet->lookPoint += Vector3d (solverManager->currentChains[sk]->positions[solverManager->currentChains[sk]->positions.size()-1].x() + rand()%150 - 50, 0, 0);
		verlet->lookChain = pair<int,int> (nn-2, nn-1);
		verlet->lookVectorResting = s->joints[19]->rotation.inverse()._transformVector(Vector3d(0,0,100));
		verlet->stiffness = (rand()%300 + 200) / 10000.0;


		sin->setPositions();
		sin2->setPositions();
		sin->longitude = sin->chain.size()/2;
		sin2->longitude = sin2->chain.size()/2;
		verlet->setPositions();
		solverManager->verlets[sk] = verlet;
		solverManager->addSolver(sin,sk);
		solverManager->addSolver(sin2,sk);

	}
}

void BeanViewer::draw() {

	if (escena->skeletons.size() > 0 && aniManager.simulationEnabled) {
		for (int sk = 0; sk < escena->skeletons.size(); ++sk) {
			skeleton* s = escena->skeletons[sk];
			Quaterniond qq = s->joints[1]->rotation.inverse();
			Quaterniond q;	q.setFromTwoVectors(qq._transformVector(Vector3d(0,1,0)), qq._transformVector(Vector3d(1,0,0)));
			s->joints[1]->addRotation(q);
			solverManager->solve(frame, this->animationPeriod(), escena->skeletons, sk);
		}

		/*skeleton* s = escena->skeletons[0];
		Vector3d v1 = s->joints[1]->getWorldPosition() - s->joints[0]->getWorldPosition();
		Vector3d v2 (1,0,0);
		Quaterniond qq = s->joints[0]->rotation.inverse();
		v1 = qq._transformVector(v1);
		v2 = qq._transformVector(v2);
		Quaterniond q;	q.setFromTwoVectors(v1,v2);
		s->joints[0]->addRotation(q);*/
	}

	// Call parent function
	escena->skinner->computeDeformations(escena->skeletons);
	AdriViewer::draw();
}