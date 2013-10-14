#include "BeanViewer.h"

BeanViewer::BeanViewer(QWidget * parent, const QGLWidget * shareWidget,
                   Qt::WindowFlags flags ) : AdriViewer(parent, shareWidget, flags) {
	initScene();
	ReBuildScene();
}

void BeanViewer::draw() {
	// Compute solvers
	if (escena->skeletons.size() > 0 && aniManager.simulationEnabled) {
		vector<Point3d> rots = escena->solverManager->computeSolvers(frame, this->animationPeriod(), escena->skeletons);
		for (int i = 0; i < escena->skeletons[0]->joints.size(); ++i)
			escena->skeletons[0]->joints[i]->addRotation(rots[i].X(), rots[i].Y(), rots[i].Z());
		escena->skeletons[0]->joints[0]->computeWorldPos();
		rots = escena->solverManager->computeVerlet(frame, this->animationPeriod(), escena->skeletons);
		for (int i = 0; i < escena->skeletons[0]->joints.size(); ++i)
			escena->skeletons[0]->joints[i]->addRotation(rots[i].X(), rots[i].Y(), rots[i].Z());
	}
	// Apply skinning
	escena->skinner->computeDeformationsWithSW(escena->skeletons);
	// Call parent function
	AdriViewer::draw();
}