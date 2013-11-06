#ifndef BEANVIEWER_H
#define BEANVIEWER_H

#include "AdriViewer.h"

#include "solvers/SolverManager.h"
#include "solvers/SolverSinusoidal.h"
#include "solvers/SolverVerlet.h"
#include "solvers/SolverStatic.h"
#include "solvers/SolverLook.h"

class BeanViewer : public AdriViewer
{
        Q_OBJECT
public:
    BeanViewer(QWidget * parent = 0, const QGLWidget * shareWidget = 0, Qt::WindowFlags flags = 0);
	void loadSolvers();

	SolverManager* solverManager;
	Particles* particles;

protected:
	virtual void draw();
};

#endif // BEANVIEWER_H
