#include "BeanViewer.h"

#include <QtCore/QDir>
#include <QtWidgets/QMessageBox>
#include <QtGui/QColor>

#include "AdriViewer.h"

#include <DataStructures/Scene.h>

#include <utils/util.h>
#include "adrimainwindow.h"

#include <iostream>
#include <fstream>

#include <QtCore/QTextStream>

BeanViewer::BeanViewer(QWidget * parent, const QGLWidget * shareWidget,
                   Qt::WindowFlags flags ) : AdriViewer(parent, shareWidget, flags) {
	initScene();
	ReBuildScene();
	solverManager = new SolverManager();
	drawLookLocators = false;
}

void BeanViewer::loadSolvers() {

	SolverVerlet *verlet = new SolverVerlet();		 // generales, el mismo para todos los esqueletos
	SolverVerlet *verlet2 = new SolverVerlet();

	// Verlet1
	verlet->index1 = 0;	verlet->index2 = 19;
	verlet->fps = 1000.0 / animationPeriod();
	verlet->data = solverManager->solverData;
	//verlet->posS = 0.5;
	verlet->hasRigid = false;
	verlet->lookSolver = false;


	verlet2->index1 = 0 ;	verlet2->index2 = 19;
	verlet2->fps = 1000.0 / animationPeriod();
	verlet2->data = solverManager->solverData;
	verlet2->hasGravity = false;
	verlet2->distS = 1;		verlet2->distD = 1;		verlet2->distStiff = 1;
	verlet2->posS = 1;		verlet2->posD = 1;		verlet2->posStiff = 1;
	verlet2->hasRigid = false;
	verlet2->lookSolver = true;

	solverManager->solverData->fps = 1000.0 / this->animationPeriod();

	int snakesPerRow = 5;
	
	for (int sk = 0; sk < escena->skeletons.size(); ++sk) {



		skeleton* s = escena->skeletons[sk];
		int n = s->joints.size();

		int row = sk / snakesPerRow;
		int col = sk % snakesPerRow;

		double x = (col - 2) * 50;
		double z = row * -75;

		Vector3d translation = Vector3d(x,0,z) - s->joints[0]->getWorldPosition();
		s->joints[0]->addTranslation(translation.x(), translation.y(), translation.z());
		s->joints[0]->addRotation(0,0,90);
		s->joints[0]->addRotation ((2 - col) * 10, 0, 0);
		s->joints[0]->addRotation (0, 0, row*10);

		//if (sk % 3 == 0) s->joints[0]->addRotation(10,0,0);
		//if (sk % 3 == 2) s->joints[0]->addRotation(-10,0,0);
		//if (sk / 3 == 0) s->joints[0]->addRotation(0,0,20);
		//if (sk / 3 == 2) s->joints[0]->addRotation(0,0,-20);
		s->joints[0]->computeWorldPos();
		solverManager->addSkeleton(sk, s);


		solverManager->brains[sk]->restUpVector = s->joints[18]->rotation.inverse()._transformVector(Vector3d(0,0,-1));

		// Create solvers to test
		SolverInit *init = new SolverInit();
		SolverDir *dir = new SolverDir();
		SolverSinusoidal *sin = new SolverSinusoidal(8 + rand()%10,2 + rand()%5,rand()%10);	
		sin->dimension = 0;		sin->longitude = 10;		sin->multAmp = 1;	sin->multFreq = 1;
		SolverLook *look = new SolverLook();

		solverManager->addSolver(init, sk);
		solverManager->addSolver(dir, sk);
		//solverManager->addSolver(sin, sk);
		solverManager->addSolver(verlet, sk);
		solverManager->addSolver(look, sk);
		solverManager->addSolver(verlet2, sk);

		solverManager->myRot = s->joints[18]->rotation;

		solverManager->brains[sk]->look = look;
		solverManager->brains[sk]->sinus = sin;

		// Init solver
		init->jt = s->joints[0];
		init->index1 = 0;	init->index2 = 18;
		init->setPositions(s);
		init->data = solverManager->solverData;
		init->initialTranslation = s->joints[0]->translation;

		verlet->addSkeleton(s, init->initialChain());
		verlet2->addSkeleton(s, init->initialChain());

		// Dir solver
		dir->data = solverManager->solverData;

		// Sin solver
		sin->index1 = 1;		sin->index2 = 19;
		sin->data = solverManager->solverData;
		sin->thresh1 = 4;	sin->thresh2 = 10;



		// Look solver
		look->index1 = 18;	look->index2 = 19;
		look->qrot = s->joints[18]->rotation;
		Vector3d v2 = s->joints[4]->translation - s->joints[3]->translation;
		look->qrot.setFromTwoVectors(s->joints[19]->translation - s->joints[18]->translation, v2).inverse();
		look->restLookVector = s->joints[18]->rotation.inverse()._transformVector(v2);
		look->data = solverManager->solverData;
		solverManager->solverData->neck = look->qrot;
		solverManager->solverData->lookPoint = Vector3d(0,480,400);
		solverManager->brains[sk]->lookPoint = s->joints[0]->translation;
		solverManager->brains[sk]->lookPoint += Vector3d(0, 480, 400);


	}

	int nodeSize = verlet->currentPositions[0].size();
	int sn = verlet->positioningStrengths.size();
	int rigids = sn / 4;
	Vector3d base = verlet->currentPositions[0][0];
	double length = (verlet->currentPositions[0][nodeSize-1] - base).norm();
	double B = 0.25;	double C = 0.85;
	double vB = 1;		double vC = 0.25;
	for (int i = 0; i < sn; ++i) {
		double p = (verlet->currentPositions[0][i] - base).norm() / length;
		if (p <= B) verlet->positioningStrengths[i] = 1;
		else if (p <= C) verlet->positioningStrengths[i] = vB + ((p - B)/(C - B))*(vC-vB);
		else verlet->positioningStrengths[i] = vC;
	}
}

void drawPointLocator(Eigen::Vector3d pt, float size, bool spot)
{
    glDisable(GL_LIGHTING);
    if(spot)
    {
        glColor3f((GLfloat)0.5, (GLfloat)0.1, (GLfloat)0.1);
        size = size*2;
    }
    else
    {
        glColor3f((GLfloat)0.1, (GLfloat)0.4, (GLfloat)0.1);
    }

    glBegin(GL_LINES);
    // queda reconstruir el cubo y ver si se pinta bien y se ha calculado correctamente.
    glVertex3f(pt.x()+size, pt.y(), pt.z());
    glVertex3f(pt.x()-size, pt.y(), pt.z());

    glVertex3f(pt.x(), pt.y()+size, pt.z());
    glVertex3f(pt.x(), pt.y()-size, pt.z());

    glVertex3f(pt.x(), pt.y(), pt.z()+size);
    glVertex3f(pt.x(), pt.y(), pt.z()-size);

    glEnd();
    glEnable(GL_LIGHTING);
}

void BeanViewer::draw() {

	if (escena->skeletons.size() > 0 && aniManager.simulationEnabled) {
		solverManager->newFrame(frame);

		for (int sk = 0; sk < escena->skeletons.size(); ++sk) {
			skeleton* s = escena->skeletons[sk];

			solverManager->update(sk, escena->skeletons[sk]);

			if (drawLookLocators) {
				Vector3d p1 = escena->skeletons[sk]->joints[solverManager->brains[sk]->look->outputs[0]->positions.size()-2]->translation;
				Vector3d p2 = solverManager->brains[sk]->look->lookPoint;
				Vector3d p3 = s->joints[18]->translation;
				glDisable(GL_LIGHTING);
				glColor3f(0.5, 0.1, 0.1);
				glBegin(GL_LINES);
				glVertex3d(p1.x(), p1.y(), p1.z());
				glVertex3d(p2.x(), p2.y(), p2.z());
				glVertex3d(p3.x(), p3.y(), p3.z());
				glVertex3d(p2.x(), p2.y(), p2.z());
				drawPointLocator(solverManager->brains[sk]->look->lookPoint, 5, true);
				glEnable(GL_LIGHTING);
			}
		}
		//solverManager->draw();
	}

	// Skinning
	for (int i = 0; i < escena->rigsArray.size(); ++i) {
		if (escena->rigsArray[i]->enableDeformation) escena->rigsArray[i]->skin->computeDeformations2(escena->skeletons[i]);
	}

	AdriViewer::draw();
}

void BeanViewer::readScene(string fileName, string name, string path) {
     QFile modelDefFile(fileName.c_str());
     if(modelDefFile.exists()) {
        modelDefFile.open(QFile::ReadOnly);
        QTextStream in(&modelDefFile);

        QString sSceneName = in.readLine(); in.readLine();
        QString sGlobalPath = in.readLine(); in.readLine();
        QString sPath = in.readLine(); in.readLine(); in.readLine();

		escena->sSceneName = sSceneName.toStdString();
		escena->sGlobalPath = sGlobalPath.toStdString();
		escena->sPath = sPath.toStdString();

		QString sModelFile = "", sSkeletonFile = "", sEmbeddingFile = "", sBindingFile = "", sRiggingFile = "";

		QStringList flags = in.readLine().split(" "); in.readLine(); in.readLine();
		if(flags.size() < 5)
			return;

		if(flags[0].toInt() != 0 && !in.atEnd())
		{
			sModelFile = in.readLine(); in.readLine(); in.readLine();
		}

		if(flags[1].toInt() != 0 && !in.atEnd())
		{
			sSkeletonFile = in.readLine(); in.readLine(); in.readLine();
		}

		if(flags[2].toInt() != 0 && !in.atEnd())
		{
			sEmbeddingFile = in.readLine(); in.readLine(); in.readLine();
		}

		if(flags[3].toInt() != 0 && !in.atEnd())
		{
			sBindingFile = in.readLine(); in.readLine(); in.readLine();
		}

		if(flags[4].toInt() != 0 && !in.atEnd())
		{
			sRiggingFile = in.readLine(); in.readLine(); in.readLine();
		}

		modelDefFile.close();

		QString newPath( path.c_str());
        newPath = newPath +"/";
        if(!sPath.isEmpty())
            newPath = newPath+"/"+sPath +"/";

		for (int i = 0; i < 1; ++i) {

			// Leer modelo
			readModel( (newPath+sModelFile).toStdString(), sSceneName.toStdString(), newPath.toStdString());

			// Constuir datos sobre el modelo
			Modelo* m = ((Modelo*)escena->models.back());
			m->sPath = newPath.toStdString(); // importante para futuras referencias
			BuildSurfaceGraphs(m);

			// Leer esqueleto
			if(!sSkeletonFile.isEmpty())
			{
				string sSkeletonFileFullPath = (newPath+sSkeletonFile).toStdString();
				readSkeletons(sSkeletonFileFullPath, escena->skeletons);
			}

			bool skinLoaded = false, riggLoaded = false;
			// Skinning
			if(!sBindingFile.isEmpty())
			{
				string sBindingFileFullPath = (newPath+sBindingFile).toStdString();//path.toStdString();
			
				//Copia del modelo para poder hacer deformaciones
				if(!m->originalModelLoaded)
					initModelForDeformation(m);
				
				loadBinding(m->bind, sBindingFileFullPath, escena->skeletons);
				skinLoaded = true;

				//escena->rig->skin->computeRestPositions(escena->skeletons);
			}

			// Load Rigging
			if(!sRiggingFile.isEmpty())
			{
			
			
				string sRiggingFileFullPath = (newPath+sRiggingFile).toStdString();//path.toStdString();

				escena->rigsArray.push_back(new Rig(scene::getNewId()));
				escena->rigsArray.back()->loadRigging(sRiggingFileFullPath);

				//escena->rig->loadRigging(sRiggingFileFullPath);

				// By now is with the skeleton, but soon will be alone
				//escena->rig->bindLoadedRigToScene(m, escena->skeletons);
				riggLoaded = true;
			}

			if(riggLoaded)
			{
				// By now is with the skeleton, but soon will be alone
				//escena->rig->bindRigToModelandSkeleton(m, escena->skeletons);
				escena->rigsArray.back()->bindRigToModelandSkeleton(m, escena->skeletons);
			}

			if(skinLoaded)
			{
				// Now it's time to do a correspondence with the loaded data and the scene.
				//escena->rig->skin->loadBindingForModel(m, escena->skeletons);

				//escena->rig->skin->computeRestPositions(escena->skeletons);

				//escena->rig->enableDeformation = true;

				escena->rigsArray.back()->skin->loadBindingForModel(m, escena->skeletons);
				escena->rigsArray.back()->skin->computeRestPositions(escena->skeletons);
				escena->rigsArray.back()->enableDeformation = true;
			}
		}
	 }
 }
