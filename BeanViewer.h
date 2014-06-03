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

#include "snake.h"

class BeanViewer : public AdriViewer
{
        Q_OBJECT
public:
    
    BeanViewer(QWidget * parent = 0, const QGLWidget * shareWidget = 0, Qt::WindowFlags flags = 0);
	
    void loadSolvers();
    virtual void initViewer();
	
    virtual void readScene(string fileName, string name, string path);

    // Solvers
	SolverManager* solverManager;
	bool drawLookLocators;

    // Particle sistem
    void initParticleScene();
    void initSceneToTestWithSnakes(int numOfSnakes = 1);

	bool paintSnakesSimulation;
	vector<snake*> serpientes;

	bool printTime;

protected:
	virtual void draw();
	virtual void animate();
};

#endif // BEANVIEWER_H
