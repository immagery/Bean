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
	particles = new Particles();
}

void BeanViewer::loadSolvers() {

	SolverVerlet *verlet = new SolverVerlet();		 // generales, el mismo para todos los esqueletos
	SolverVerlet *verlet2 = new SolverVerlet();

	// Verlet1
	verlet->index1 = 0;	verlet->index2 = 19;
	verlet->fps = 1000.0 / animationPeriod();
	verlet->data = solverManager->solverData;

	verlet2->index1 = 18;	verlet2->index2 = 19;
	verlet2->fps = 1000.0 / animationPeriod();
	verlet2->data = solverManager->solverData;
	verlet2->hasGravity = false;

	solverManager->solverData->fps = 1000.0 / this->animationPeriod();

	
	for (int sk = 0; sk < escena->skeletons.size(); ++sk) {

		skeleton* s = escena->skeletons[sk];
		int n = s->joints.size();
		Vector3d translation = Vector3d((sk % 3) * 75, 0, -200 + (sk / 3)*100) - s->joints[0]->getWorldPosition();
		s->joints[0]->addTranslation(translation.x(), translation.y(), translation.z());
		s->joints[0]->addRotation(0,0,90);
		if (sk % 3 == 0) s->joints[0]->addRotation(20,0,0);
		if (sk % 3 == 2) s->joints[0]->addRotation(-20,0,0);
		if (sk / 3 == 0) s->joints[0]->addRotation(0,0,20);
		if (sk / 3 == 2) s->joints[0]->addRotation(0,0,-20);
		s->joints[0]->computeWorldPos();
		solverManager->addSkeleton(sk, s);

		// Create solvers to test
		SolverInit *init = new SolverInit();
		SolverDir *dir = new SolverDir();
		SolverSinusoidal *sin = new SolverSinusoidal(10,4,rand()%10);	sin->dimension = 0;		sin->longitude = 10;
		SolverLook *look = new SolverLook();

		solverManager->addSolver(init, sk);
		solverManager->addSolver(dir, sk);
		solverManager->addSolver(sin, sk);
		solverManager->addSolver(verlet, sk);
		solverManager->addSolver(look, sk);
		solverManager->addSolver(verlet2, sk);

		verlet->addSkeleton(s);
		verlet2->addSkeleton(s);

		// Init solver
		init->jt = s->joints[0];
		//init->restTranslation = s->joints[0]->translation;
		init->data = solverManager->solverData;
		for (int i = 0; i <= s->joints.size()-6; ++i) {
			init->initPositions.push_back(s->joints[i]->pos);
			init->initQORIENTS.push_back(s->joints[i]->qOrient);
			init->initQROTS.push_back(s->joints[i]->qrot);
		}
		init->initialTranslation = s->joints[0]->translation;

		// Dir solver
		dir->data = solverManager->solverData;

		// Sin solver
		sin->index1 = 1;		sin->index2 = 19;
		sin->data = solverManager->solverData;
		sin->thresh1 = 4;	sin->thresh2 = 10;



		// Look solver
		look->index1 = 18;	look->index2 = 19;
		look->qrot = s->joints[18]->rotation;
		look->qrot.setFromTwoVectors(s->joints[19]->translation - s->joints[18]->translation, Vector3d(0,1,0)).inverse();
		look->restLookVector = s->joints[18]->rotation.inverse()._transformVector(Vector3d(0,1,0));
		look->data = solverManager->solverData;
		solverManager->solverData->neck = look->qrot;
		solverManager->solverData->lookPoint = Vector3d(0,480,400);

	}

		int sn = verlet->positioningStrengths.size();
		for (int i = 0; i < verlet->positioningStrengths.size(); ++i) {
			verlet->positioningStrengths[i] = (1 - ((double)i / (verlet->positioningStrengths.size() + 1))) * 1;
		}
		verlet->positioningStrengths[0] *= 2;
		verlet->positioningStrengths[1] *= 1.5;
		verlet->positioningStrengths[2] *= 1.2;
		verlet->positioningStrengths[3] *= 1.15;
		verlet->positioningStrengths[4] *= 1.07;
		verlet->positioningStrengths[5] *= 1.03;
		verlet->positioningStrengths[6] *= 1.01;
		//verlet->positioningStrengths[1] = verlet->positioningStrengths[2] = 1;
		verlet2->positioningStrengths[0] = 3;
		verlet2->positioningStrengths[1] = 3;

}

void BeanViewer::draw() {

	if (escena->skeletons.size() > 0 && aniManager.simulationEnabled) {
		solverManager->newFrame(frame);
		for (int sk = 0; sk < escena->skeletons.size(); ++sk) {
			solverManager->update(sk, escena->skeletons[sk]);
		}
		solverManager->draw();
	}

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

		for (int i = 0; i < 9; ++i) {

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
