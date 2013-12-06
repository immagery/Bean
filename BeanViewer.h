#ifndef BEANVIEWER_H
#define BEANVIEWER_H

#include "AdriViewer.h"
#include <QtOpenGL/QGLWidget>
#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>
#include <QGLViewer/qglviewer.h>

#include "QGLViewer/constraint.h"

#include "solvers/SolverManager.h"
#include "solvers/AllSolvers"

class BeanViewer : public AdriViewer
{
        Q_OBJECT
public:
    BeanViewer(QWidget * parent = 0, const QGLWidget * shareWidget = 0, Qt::WindowFlags flags = 0);
	void loadSolvers();
	virtual void readScene(string fileName, string name, string path);

	SolverManager* solverManager;
	bool drawLookLocators;


protected:
	virtual void draw();
};

#endif // BEANVIEWER_H
