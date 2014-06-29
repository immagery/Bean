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

#include <DataStructures\Particle.h>
#include <DataStructures\pConstraintDistance.h>
#include <DataStructures\pConstraintAngle.h>

#define PRINT_STEP_PERFORMANCE false


BeanViewer::BeanViewer(QWidget * parent, const QGLWidget * shareWidget,
                   Qt::WindowFlags flags ) : AdriViewer(parent, shareWidget, flags) {

	solverManager = new SolverManager();
	printTime = true;
	drawLookLocators = false;
	
	float snakeLength = 45;
	float snakeWidth = 8;
	float headHeight = 3;
	int rows = 1;
	int cols = 1; 
	int numSnakes = rows*cols;

	paintSnakesSimulation = false;

	// Read several snakes from disk
	initSceneToTestWithSnakes(numSnakes);

	Vector3d gridOrigin((rows*snakeLength)/2.0,0,(cols*snakeWidth)/2.0);

	for(int i = 0; i < rows; i++)
	{
		for(int j = 0; j < cols; j++)
		{
			int idSnake = i*cols+j;

			Modelo* m = ((Modelo*)escena->models[idSnake]);
			skeleton* skt = ((skeleton*)escena->skeletons[idSnake]);

			serpientes.push_back(new snake());
			serpientes.back()->initFromSkeleton(m, skt);

			float newHeadHeight = headHeight * ((float)rand())/RAND_MAX * 2;

			Vector3d origen(i*snakeLength, newHeadHeight ,j*snakeWidth);
			Vector3d position = origen-gridOrigin;

			printf("Snake %d (%d,%d): en %f %f %f\n", idSnake, i, j, position.x(), position.y(), position.z());

			serpientes.back()->moveToPosition(position);

			Vector3d or(((double)rand())/RAND_MAX*2-1, 0, ((double)rand())/RAND_MAX*2-1);
			or.normalize();
			if(or.norm() == 0) or = Vector3d(1,0,0);
			serpientes.back()->changeOrientation(or);
		}
	}



	//initParticleScene();
	startAnimation();

	initViewer();

	initScene();

}

void BeanViewer::initSceneToTestWithSnakes(int numOfSnakes)
{

	// Inicialización a saco
	QString sFileName = "snake_scene.txt";
	QString sPrefix = "snake_simple";
	QString sFilePath = "C:\\Users\\chus\\Documents\\dev\\Data\\models\\";

	for(int i = 0; i < numOfSnakes; i++)
	{
		readScene(sFileName.toStdString(), sPrefix.toStdString(), sFilePath.toStdString());
	}

	//printf("\n[BeanViewer]LOG: abrimos directamente el fichero de una serpiente\n");
}

void BeanViewer::initParticleScene()
{
	int rows = 1;
	int cols = 1;

	Vector3d gridOrigin((rows*550.0)/2.0,0,(cols*250.0)/2.0);

	for(int i = 0; i < rows; i++)
	{
		for(int j = 0; j < cols; j++)
		{
			Vector3d origen(i*550, 50 ,j*250);
			
			serpientes.push_back(new snake());
			serpientes.back()->initGeneric(origen-gridOrigin);

		}
	}
}

void BeanViewer::initViewer()
{
	printf("initViewer\n");

    setGridIsDrawn(false);
    setAxisIsDrawn(false);
    setFPSIsDisplayed(true);
	setBackgroundColor(QColor(Qt::darkGray));

	setSceneCenter(qglviewer::Vec(0,0,0));
	setSceneRadius(1200);

	qglviewer::Vec min(-400,-400,-400); qglviewer::Vec max(400,400,400);
	setSceneBoundingBox(min, max);
	showEntireScene();
	updateGL();
}

void BeanViewer::loadSolvers() {

	SolverVerlet *verlet = new SolverVerlet(4);		 // generales, el mismo para todos los esqueletos
	SolverVerlet *verlet2 = new SolverVerlet(4);		
	

	// Verlet1
	verlet->index1 = 0;	verlet->index2 = 19;
	verlet->fps = 1000.0 / animationPeriod();
	verlet->data = solverManager->solverData;
	verlet->hasRigid = false;
	verlet->lookSolver = false;
	verlet->nextVerlet = verlet2;

	// Verlet2
	verlet2->index1 = 0 ;	verlet2->index2 = 19;
	verlet2->fps = 1000.0 / animationPeriod();
	verlet2->data = solverManager->solverData;
	verlet2->hasGravity = false;
	verlet2->hasRigid = false;
	verlet2->lookSolver = true;
	verlet2->nextVerlet = verlet;

	solverManager->solverData->fps = 1000.0 / this->animationPeriod();

	int snakesPerRow = 1;
	
	for (int sk = 0; sk < escena->skeletons.size(); ++sk) {

		skeleton* s = escena->skeletons[sk];
		
		/*
		int n = s->joints.size();

		int row = sk / snakesPerRow;
		int col = sk % snakesPerRow;
		row = 0;
		col = 2;

		double x = (col - 2) * 50;
		double z = row * -75;

		
		Vector3d translation;
		if (sk == 0) translation = Vector3d(0,0,-50) - s->joints[0]->getWorldPosition();
		if (sk == 1) translation = Vector3d(0,0,0) - s->joints[0]->getWorldPosition();
		if (sk == 2) translation = Vector3d(0,0,50) - s->joints[0]->getWorldPosition();
		if (sk == 3) translation = Vector3d(20,0,-25) - s->joints[0]->getWorldPosition();
		if (sk == 4) translation = Vector3d(20,0,25) - s->joints[0]->getWorldPosition();

		if (sk == 5) translation = Vector3d(-20,0,-75) - s->joints[0]->getWorldPosition();
		if (sk == 6) translation = Vector3d(-20,0,+75) - s->joints[0]->getWorldPosition();
		if (sk == 7) translation = Vector3d(0,0,100) - s->joints[0]->getWorldPosition();
		if (sk == 8) translation = Vector3d(-20,0,-25) - s->joints[0]->getWorldPosition();
		if (sk == 9) translation = Vector3d(-20,0,25) - s->joints[0]->getWorldPosition();

		s->joints[0]->addTranslation(translation.x(), translation.y(), translation.z());
		s->joints[0]->addRotation(0,0,90);
		*/
		
		// Posicinamos todos los joints al inicio.
		s->joints[0]->computeWorldPos();
		
		// Añadimos el esqueleto al conjunto de elementos a resolver.
		solverManager->addSkeleton(sk, s);

		// Especificamos un restUpVector...
		solverManager->brains[sk]->restUpVector = s->joints[18]->rotation.inverse()._transformVector(Vector3d(0,0,-1));

		// Create solvers to test

		// Init solver
		SolverInit *init = new SolverInit(0);	
			
		init->index1 = 0;	// Skt joint idx 1
		init->index2 = 18;  // Skt joint idx 2

		//TODEBUG -> interesa poner mas puntos?, he eliminado el final como media.
		init->setPositions(s);
		init->initialize();

		// Conexion cerrando el bucle de simulacion
		// TO_DO
		solverManager->addSolver(init, sk);
		
		SolverPos *pos = new SolverPos(0);
		pos->index1 = 0;
		pos->index2 = 19;
		pos->restTrans = s->joints[0]->translation;
		//solverManager->addSolver(pos,sk);
		
		SolverDir *dir = new SolverDir(1);			
		
		SolverSinusoidal *sin = new SolverSinusoidal(4 + rand()%10,2 + rand()%2,rand()%10, 2);		
		sin->dimension = 0;		
		sin->longitude = 10;		
		sin->multAmp = 1;	
		sin->multFreq = 1;

		// Sin solver
		sin->index1 = 1;		
		sin->index2 = 19;
		sin->thresh1 = 10;	
		sin->thresh2 = 15;	
		sin->thresh3 = -1;
		
		SolverSinusoidal *sin2 = new SolverSinusoidal(8 + rand()%10,2 + rand()%3,rand()%10, 2);		
		sin2->dimension = 2;		
		sin2->longitude = 10;		
		sin2->multAmp = 1;	
		sin2->multFreq = 1;

		solverManager->addSolver(verlet, sk);

		// Head solver
		SolverHead *head = new SolverHead(4);		
		head->desiredPos = Vector3d(0,480,480);
		//head->data = solverManager->solverData;
		head->index1 = 18;	head->index2 = 19;
		//solverManager->addSolver(head,sk);


		// Look solver		
		SolverLook *look = new SolverLook(3);
		/*		
		look->index1 = 18;	
		look->index2 = 19;
		look->qrot = s->joints[18]->rotation;
		Vector3d v2 = s->joints[4]->translation - s->joints[3]->translation;
		look->qrot.setFromTwoVectors(s->joints[19]->translation - s->joints[18]->translation, v2).inverse();
		look->restLookVector = s->joints[18]->rotation.inverse()._transformVector(v2);
		//look->data = solverManager->solverData;
		look->restDistance = (init->initialChain()->positions[19] - init->initialChain()->positions[18]).norm();
		solverManager->solverData->neck = look->qrot;
		solverManager->solverData->lookPoint = Vector3d(0,800,0);
		solverManager->brains[sk]->lookPoint = s->joints[0]->translation;
		solverManager->brains[sk]->lookPoint += Vector3d(0, 480, 0);
		*/
		
		
		//solverManager->addSolver(look, sk);

		
		solverManager->addSolver(verlet2, sk);
		init->inputs[0] = (verlet2->outputs[sk]);

		//solverManager->myRot = s->joints[18]->rotation;

		//solverManager->brains[sk]->look = look;
		//solverManager->brains[sk]->sinus = sin;
		//solverManager->brains[sk]->headPosition = head;


		verlet->addSkeleton(s, init->initialChain());
		verlet2->addSkeleton(s, init->initialChain());

		// Dir solver
		//dir->data = solverManager->solverData;

		/*sin2->index1 = 1;	sin2->index2 = 19;
		sin2->data = solverManager->solverData;
		sin2->thresh1 = 10;	sin2->thresh2 = 15;	sin2->thresh3 = -1;*/

	}

	int nodeSize = verlet->currentPositions[0].size();
	int sn = verlet->positioningStrengths.size();
	int rigids = sn / 4;
	Vector3d base = verlet->currentPositions[0][0];
	double length = (verlet->currentPositions[0][nodeSize-1] - base).norm();
	double B = 0.25;	double C = 0.85;
	double vB = 1;		double vC = 0.25;

	for (int i = 0; i < sn; ++i) 
	{
		double p = (verlet->currentPositions[0][i] - base).norm() / length;
		if (p <= B) 
			verlet->positioningStrengths[i] = 1;

		else if (p <= C) 
			verlet->positioningStrengths[i] = vB + ((p - B)/(C - B))*(vC-vB);

		else verlet->positioningStrengths[i] = vC;
	}

	for (int i = 0; i < sn; ++i) 
	{
		if (i < 10) 
			verlet->rigidnessStrengths[i] = 1;
		else		
			verlet->rigidnessStrengths[i] = 0;
	}

	for (int i = 0; i < verlet2->currentPositions[0].size(); ++i)
	 {
		verlet2->positioningStrengths[i] = (double)i / verlet2->currentPositions[0].size();
		//verlet2->positioningStrengths[i] = 0;
		//if (i == verlet2->currentPositions[0].size()-1) verlet2->positioningStrengths[i] = 1;
	}
}
/*
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
*/

void BeanViewer::animate()
 {
	//TODO - timesteping
	clock_t end, start;
	start = clock();
	
	for(int id = 0; id< serpientes.size(); id++)
		serpientes[id]->move();

	end = clock();

	if(printTime)
	{
		printf("SimulationTime: %f\n", ((double)(end-start))/CLOCKS_PER_SEC);
		
	}
	printTime = false;

	AdriViewer::animate();
 }
void BeanViewer::draw() 
{
	clock_t end, start;

	if(PRINT_STEP_PERFORMANCE) start = clock();

	/*
	if (escena->skeletons.size() > 0 && aniManager.simulationEnabled) 
	{
		solverManager->newFrame(frame);

		
		for (int sk = 0; sk < escena->skeletons.size(); ++sk) 
		{
			skeleton* s = escena->skeletons[sk];
			solverManager->update(sk, escena->skeletons[sk]);

			if (drawLookLocators) 
			{
				if(!solverManager->brains[sk]->look) continue;

				int size = solverManager->brains[sk]->look->outputs[0]->positions.size();
					
				Vector3d p1 = escena->skeletons[sk]->joints[size-2]->translation;
				Vector3d p2 = solverManager->brains[sk]->look->lookPoint;
				Vector3d p3 = s->joints[18]->translation;
				Vector3d p4 = escena->skeletons[sk]->joints[size-2]->translation;
				Vector3d p5 = escena->skeletons[sk]->joints[size-1]->translation;
				Vector3d look = (p5 - p4).normalized();
				Vector3d pf = p4 + look*100;

				glDisable(GL_LIGHTING);
				glColor3f(0.5, 0.1, 0.1);
				glBegin(GL_LINES);
				glVertex3d(p1.x(), p1.y(), p1.z());
				glVertex3d(p2.x(), p2.y(), p2.z());
				//glVertex3d(p3.x(), p3.y(), p3.z());
				//glVertex3d(p2.x(), p2.y(), p2.z());
				//glVertex3d(p4.x(), p4.y(), p4.z());
				//glVertex3d(pf.x(), pf.y(), pf.z());
				drawPointLocator(solverManager->brains[sk]->look->lookPoint, 5, true);
				glEnable(GL_LIGHTING);
			}
			
		}

		solverManager->draw();
		glDisable(GL_LIGHTING);
		glColor3f(1,1,1);
		Vector3d p1 = escena->skeletons[0]->joints[0]->translation;	p1.x() -= 200;	p1.z() += 200;

		Vector3d p2 = p1;	p2.x() += 1000;
		Vector3d p3 = p2;	p3.z() -= 1000;
		Vector3d p4 = p3;	p4.x() -= 1000;

		glBegin(GL_QUADS);
		glVertex3d(p1.x(), p1.y(), p1.z());
		glVertex3d(p2.x(), p2.y(), p2.z());
		glVertex3d(p3.x(), p3.y(), p3.z());
		glVertex3d(p4.x(), p4.y(), p4.z());
		glEnd();

	}
	*/

	glDisable(GL_LIGHTING);
	if(paintSnakesSimulation)
	{
		for(int id = 0; id< serpientes.size(); id++)
			serpientes[id]->drawFunc();
	}
	

	// Base plane for references.
	
	int rows = 0;
	int cols = 5;

	float length = 100;


	Vector3d orig(-length/2.0, -2, -length/2.0);
	float maxX = length/rows;
	float maxZ = length/cols;

	glColor3f(0.9,0.9,0.9);
	glBegin(GL_QUADS);
	for(int i = 0; i< rows; i++)
	{
		for(int j = 0; j< cols; j++)
		{
			if((i+j) % 2 == 0)
				glColor3f(0.2,0.2,0.2);
			else
				glColor3f(0.9,0.9,0.9);

			Vector3d newOrig = orig + Vector3d(maxX*i, 0 ,maxZ*j);
			glNormal3d(0,1.0,0);
			glVertex3d(newOrig.x(), newOrig.y(), newOrig.z() + maxZ);
			glVertex3d(newOrig.x() + maxX, newOrig.y(), newOrig.z() + maxZ);
			glVertex3d(newOrig.x() + maxX, newOrig.y(), newOrig.z());
			glVertex3d(newOrig.x(), newOrig.y(), newOrig.z());
			
		}
	}
	glEnd();
	
	glEnable(GL_LIGHTING);

	if(PRINT_STEP_PERFORMANCE) 
	{
		end = clock();
		printf("simulation and control drawing: %f\n", timelapse(start,end));
		start = clock();
	}

	// Skinning
	for (int i = 0; i < escena->rigsArray.size(); ++i) {
		if (escena->rigsArray[i]->enableDeformation) escena->rigsArray[i]->skin->computeDeformations2(escena->skeletons[i]);
	}
	
	if(PRINT_STEP_PERFORMANCE) 
	{
		end = clock();
		printf("Deformation time: %f\n", timelapse(start,end));
		start = clock();
	}

	AdriViewer::draw();
	
	if(PRINT_STEP_PERFORMANCE) 
	{
		end = clock();
		printf("Model drawing: %f\n", timelapse(start,end));
	}

	int stopBreakpoint = 0;
	stopBreakpoint++;
	stopBreakpoint++;
}

void BeanViewer::readScene(string fileName, string name, string path) 
{
	 QFile modelDefFile(fileName.c_str());
     if(modelDefFile.exists()) 
	 {
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

		//for (int i = 0; i < 5; ++i) 
		{
			// Leer modelo
			readModel( (newPath+sModelFile).toStdString(), sSceneName.toStdString(), newPath.toStdString());

			// Constuir datos sobre el modelo
			Modelo* m = ((Modelo*)escena->models.back());
			m->sPath = newPath.toStdString(); // importante para futuras referencias
			BuildSurfaceGraphs(m);

			m->shading->colors.resize(m->vn());
			for (int i = 0; i < m->vn(); ++i) 
			{
				m->shading->colors[i].resize(3);
				Vector3d p = m->nodes[i]->position;
				if ((int)(p.x()/2) % 2 == 0) 
				{
					m->shading->colors[i][0] = 0.4;
					m->shading->colors[i][1] = 0.9;
					m->shading->colors[i][2] = 0.4;
				} 
				else 
				{
					m->shading->colors[i][0] = 0.2;
					m->shading->colors[i][1] = 0.4;
					m->shading->colors[i][2] = 0.2;
				}
			}

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
