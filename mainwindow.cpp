//#include <cmath>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtWidgets/QToolButton>
#include <QtWidgets/QToolBar>
#include <QtCore/QDir>
#include <QtWidgets/QFileDialog>

#include <QtWidgets/QTreeView>
#include <QtGui/QStandardItem>

#include <gui/treeitem.h>
#include "treemodel.h"

#include "outliner.h"
#include "DataStructures/modelo.h"
#include "DataStructures/Object.h"

#include "utils/util.h"

#include "globalDefinitions.h"

#include "BeanViewer.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    AdriMainWindow(parent)
    //ui(new Ui::MainWindow)
{

	ui->setupUi(this);
	ui->verticalLayout_4->removeWidget(ui->glCustomWidget);
	delete ui->glCustomWidget;
	ui->glCustomWidget = new BeanViewer(ui->frame);
	ui->glCustomWidget->parent = this;
	setViewer();
	connectSignals();

	connect(ui->setupSolvers, SIGNAL(clicked()), this, SLOT(loadSolvers()));
	connect(ui->useSolvers, SIGNAL(toggled(bool)), this, SLOT(toggleSolvers(bool))); 
	connect(ui->twistCorrection, SIGNAL(toggled(bool)), this, SLOT(toggleTwistCorrection(bool)));

	connect(ui->gravitySlider, SIGNAL(valueChanged(int)), this, SLOT(changeVerletGravity(int)));

	connect(ui->lookX, SIGNAL(valueChanged(int)), this, SLOT(changeLookX(int)));
    connect(ui->lookY, SIGNAL(valueChanged(int)), this, SLOT(changeLookY(int)));
    connect(ui->lookZ, SIGNAL(valueChanged(int)), this, SLOT(changeLookZ(int)));

	connect(ui->drawLocatorsCB, SIGNAL(toggled(bool)), this, SLOT(changedDrawLocators(bool)));
	connect(ui->behaviourCombo, SIGNAL(	currentIndexChanged(int)), this, SLOT(changeBehaviour(int)));

	connect(ui->snakeSelector, SIGNAL(currentIndexChanged(int)), this, SLOT(selectSnake(int)));

	connect(ui->oscAmplitude, SIGNAL(valueChanged(int)), this, SLOT(changeOscAmplitude(int)));
	connect(ui->oscFrequency, SIGNAL(valueChanged(int)), this, SLOT(changeOscAmplitude(int)));
	connect(ui->oscThresh1, SIGNAL(valueChanged(int)), this, SLOT(changeOscThresh1(int)));

	connect(ui->lookRadius, SIGNAL(valueChanged(int)), this, SLOT(changeLookPointRadius(int)));

	connect(ui->verletRigidness, SIGNAL(toggled(bool)), this, SLOT(toggleVerletRigidness(bool)));

	connect(ui->twistPropagationSlider, SIGNAL(valueChanged(int)), this, SLOT(changeTwistPropagation(int)));
	connect(ui->twistSmoothingSlider, SIGNAL(valueChanged(int)), this, SLOT(changeTwistSmoothing(int)));

	connect(ui->ksSlider, SIGNAL(valueChanged(int)), this, SLOT(changeSpringParameters(int)));
	connect(ui->kdSlider, SIGNAL(valueChanged(int)), this, SLOT(changeSpringParameters(int)));
	connect(ui->stiffSlider, SIGNAL(valueChanged(int)), this, SLOT(changeSpringParameters(int)));

	connect(ui->buildAttackCurve, SIGNAL(clicked()), this, SLOT(buildAttackCurve()));

	connect(ui->headX, SIGNAL(valueChanged(int)), this, SLOT(changeHeadPosition(int)));
	connect(ui->headY, SIGNAL(valueChanged(int)), this, SLOT(changeHeadPosition(int)));
	connect(ui->headZ, SIGNAL(valueChanged(int)), this, SLOT(changeHeadPosition(int)));

	connect(ui->moveButton, SIGNAL(clicked()), this, SLOT(toggleMove()));
	connect(ui->scaleButton, SIGNAL(clicked()), this, SLOT(scaleCurve()));

	lastX = lastY = lastZ = 0;
	lastLX = lastLY = lastLZ = 0;
	lastOsc = 0;
    
}

void MainWindow::scaleCurve() {
	SolverManager* manager = ((BeanViewer*)(ui->glCustomWidget))->solverManager;
	SolverVerlet* verlet = (SolverVerlet*) (manager->solvers[0][manager->solvers[0].size()-1]);
	verlet->scaleCurve();
}

void MainWindow::changeHeadPosition(int) {
	SolverManager* manager = ((BeanViewer*)(ui->glCustomWidget))->solverManager;
	manager->solverData->desiredPos = Vector3d(ui->headX->value(), ui->headY->value(), ui->headZ->value());
}

void MainWindow::toggleMove() {
	SolverManager* manager = ((BeanViewer*)(ui->glCustomWidget))->solverManager;
	SolverVerlet* verlet = (SolverVerlet*) (manager->solvers[0][manager->solvers[0].size()-1]);
	verlet->buildAttackCurves();
	SolverHead* h = (SolverHead*) (manager->solvers[0][manager->solvers[0].size()-2]);
	h->moving = !(h->moving);
}

void MainWindow::buildAttackCurve() {
	SolverManager* manager = ((BeanViewer*)(ui->glCustomWidget))->solverManager;
	SolverVerlet* verlet = (SolverVerlet*) (manager->solvers[0][manager->solvers[0].size()-1]);
	//verlet->buildAttackCurves();
	//ui->buildAttackCurve->setEnabled(false);
	verlet->moveFlag = !verlet->moveFlag;
}

void MainWindow::changeSpringParameters(int) {
	SolverManager* manager = ((BeanViewer*)(ui->glCustomWidget))->solverManager;
	SolverVerlet* verlet = (SolverVerlet*) (manager->solvers[0][manager->solvers[0].size()-1]);
	double ui1 = ui->ksSlider->value();
	double ui2 = ui->kdSlider->value() / 100.0;
	double ui3 = ui->stiffSlider->value() / 100.0;
	verlet->distS = ui->ksSlider->value();
	verlet->distD = ui->kdSlider->value() / 100.0;
	verlet->distStiff = ui->stiffSlider->value() / 100.0;
}

void MainWindow::changeOscThresh1(int) {
	/*SolverManager* manager = ((BeanViewer*)(ui->glCustomWidget))->solverManager;
	SolverVerlet* verlet = (SolverVerlet*) (manager->solvers[0][manager->solvers[0].size()-1]);
	int inc = ui->oscThresh1->value() - lastOsc;
	if (inc < 0) {
		verlet->disableAttack();
		ui->buildAttackCurve->setEnabled(true);
	}
	lastOsc = ui->oscThresh1->value();*/
	((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->alpha = ui->oscThresh1->value() / 1000.0;
	//numTwisted = ui->twistPropagationSlider->value();
	/*
	int lastPos2 = verlet->currentPositions[0].size()-1;
	SolverHead* head = (SolverHead*) (manager->solvers[0][manager->solvers[0].size()-2]);
	Vector3d look = (verlet->currentPositions[0][lastPos2] - verlet->currentPositions[0][lastPos2-1]).normalized();
	head->outputs[0]->positions[head->outputs[0]->positions.size()-1] += inc * look;
	return;*/

	//int lastPos = verlet->curves[0].size()-1;
	//int lastPos2 = verlet->currentPositions[0].size()-1;
	//Vector3d look = (verlet->curves[0][lastPos] - verlet->curves[0][lastPos-1]).normalized();
	//Vector3d look = (verlet->currentPositions[0][verlet->currentPositions[0].size()-1] - verlet->currentPositions[0][verlet->currentPositions[0].size()-2]).normalized();
	//Vector3d look = (verlet->currentPositions[0][lastPos2] - verlet->currentPositions[0][lastPos2-1]).normalized();
	//verlet->currentPositions[0][verlet->currentPositions[0].size()-1] += ui->oscThresh1->value() * look;
	//verlet->idealPositions[0][lastPos] += inc * look;
}

void MainWindow::changeTwistPropagation(int) {
	((BeanViewer*)(ui->glCustomWidget))->solverManager->numTwisted = ui->twistPropagationSlider->value();
}

void MainWindow::changeTwistSmoothing(int) {
	((BeanViewer*)(ui->glCustomWidget))->solverManager->smoothingIterations = ui->twistSmoothingSlider->value();
}

void MainWindow::toggleVerletRigidness(bool) {
	((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->rigidness = ui->verletRigidness->isChecked();
}

void MainWindow::changeLookPointRadius(int) {
	int newRadius = ui->lookRadius->value();
	int selectedSnake = ui->snakeSelector->currentIndex();
	if (selectedSnake == 0) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->brains.size(); ++i) 
		{
			Intelligence* brain = ((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[i];
			
			brain->globalLookPointRadius = newRadius;
			brain->globalThita = fRand(0, M_PI/6.0);
			brain->globalPhi = fRand(0, 2.0*M_PI);
		}
	} else {
		((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[selectedSnake-1]->lookPointRadius = newRadius;
	}
}

void MainWindow::changeOscAmplitude(int) {
	int selectedSnake = ui->snakeSelector->currentIndex();
	double value = ui->oscAmplitude->value() / 10.0;
	((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->ampMultiplier = value;
	if (selectedSnake == 0) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->brains.size(); ++i)
			((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[i]->ampMultiplier = value;
	}
	else ((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[selectedSnake-1]->ampMultiplier = value;
}

void MainWindow::changeOscFrequency(int) {
	int selectedSnake = ui->snakeSelector->currentIndex();
	double value = ui->oscFrequency->value() / 10.0;
	if (selectedSnake == 0) {
		((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->freqMultiplier = value;
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->brains.size(); ++i)
			((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[i]->freqMultiplier = value;
	}
	else ((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[selectedSnake-1]->freqMultiplier = value;
}

void MainWindow::selectSnake(int) {
	ui->oscAmplitude->blockSignals(true);
	int selectedSnake = ui->snakeSelector->currentIndex();
	if (selectedSnake == 0) {
		ui->oscAmplitude->setValue(((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->ampMultiplier*10);
		ui->oscFrequency->setValue(((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->freqMultiplier*10);
		ui->lookRadius->setValue(((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[0]->globalLookPointRadius);
	} else if (selectedSnake > 0) {
		ui->oscAmplitude->setValue(((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[selectedSnake-1]->ampMultiplier * 10);
		ui->oscFrequency->setValue(((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[selectedSnake-1]->freqMultiplier * 10);
		ui->lookRadius->setValue(((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[selectedSnake-1]->lookPointRadius);
	}
	ui->oscAmplitude->blockSignals(false);
}

void MainWindow::changeBehaviour(int) {
	int index = ui->behaviourCombo->	currentIndex();
	for (int i = 0; i < ((BeanViewer*)ui->glCustomWidget)->solverManager->brains.size(); ++i) {
		((BeanViewer*)ui->glCustomWidget)->solverManager->brains[i]->setState((Intelligence::States)index);
	}
}

void MainWindow::changedDrawLocators(bool) {
	((BeanViewer*)ui->glCustomWidget)->drawLookLocators = ui->drawLocatorsCB->isChecked();
}

void MainWindow::changeVerletStiffness(int) {
	/*for (int i = 0; i < ui->glCustomWidget->escena->skeletons.size(); ++i) {
	}*/
}

void MainWindow::changeVerletGravity(int) {
	((BeanViewer*)ui->glCustomWidget)->solverManager->solverData->gravity = ui->gravitySlider->value();
}

void MainWindow::setViewer() {
    ui->glCustomWidget->setObjectName(QStringLiteral("glCustomWidget"));
    QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(ui->glCustomWidget->sizePolicy().hasHeightForWidth());
    ui->glCustomWidget->setSizePolicy(sizePolicy);
	ui->verticalLayout_4->addWidget(ui->glCustomWidget);
}

void MainWindow::loadSolvers() {
    ((BeanViewer*)(ui->glCustomWidget))->loadSolvers();
	ui->setupSolvers->setEnabled(false);
	//ui->useVerlet->setCheckable(true);
	ui->useSolvers->setCheckable(true);

	ui->snakeSelector->removeItem(0);
	ui->snakeSelector->addItem("All");

	for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->solvers.size(); ++i) {
		QString text = QString::number((i));
		ui->snakeSelector->addItem(text);
	}
}

void MainWindow::toggleTwistCorrection(bool) {
	((BeanViewer*)(ui->glCustomWidget))->solverManager->twistCorrectionEnabled = ui->twistCorrection->isChecked();
	//((BeanViewer*)(ui->glCustomWidget))->solverManager->hasVerlet = b;
}

void MainWindow::toggleSolvers(bool) {
	//bool b = ui->useVerlet->isChecked();
	//((BeanViewer*)(ui->glCustomWidget))->solverManager->oscillation = b;
}

void MainWindow::changeLookX(int) {
	int increment = ui->lookX->value() - lastLX;
	lastLX = ui->lookX->value();
	int index = ui->snakeSelector->currentIndex();
	
	((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->lookPoint.x() += increment/10.0;

	if (index == 0) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->brains.size(); ++i) {
			((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[i]->globalLookPoint.x() += increment/10.0;
		}
	} else {
		((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[index-1]->lookPoint.x() += increment/10.0;
	}
}

void MainWindow::changeLookY(int) {
	int increment = ui->lookY->value() - lastLY;
	lastLY = ui->lookY->value();
	int index = ui->snakeSelector->currentIndex();

	((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->lookPoint.y() += increment/10.0;

	if (index == 0) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->brains.size(); ++i) {
			((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[i]->globalLookPoint.y() += increment/10.0;
		}
	} else {
		((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[index-1]->lookPoint.y() += increment/10.0;
	}
}

void MainWindow::changeLookZ(int) {
	int increment = ui->lookZ->value() - lastLZ;
	lastLZ = ui->lookZ->value();
	int index = ui->snakeSelector->currentIndex();

	((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->lookPoint.z() += increment/10.0;

	if (index == 0) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->brains.size(); ++i) {
			((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[i]->globalLookPoint.z() += increment/10.0;
		}
	} else {
		((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[index-1]->lookPoint.z() += increment/10.0;
	}
}

void MainWindow::changeTransformRotateAmountX(int) {
	rotationX =  ((float)ui->dialX->value()/10.0);	

	Quaterniond newRot = fromEulerAngles(rotationX, rotationY, rotationZ);
	((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->baseRotation = newRot;

    QString msg = QString::number((rotationX));
    ui->rotationEditX->setText(msg);
}
    
void MainWindow::changeTransformRotateAmountY(int) {
	rotationY =  ((float)ui->dialY->value()/10.0);	

	Quaterniond newRot = fromEulerAngles(rotationX, rotationY, rotationZ);
	((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->baseRotation = newRot;

    QString msg = QString::number((rotationY));
    ui->rotationEditY->setText(msg);
}
    
void MainWindow::changeTransformRotateAmountZ(int) {
	rotationZ =  ((float)ui->dialZ->value()/10.0);

	Quaterniond newRot = fromEulerAngles(rotationX, rotationY, rotationZ);
	((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->baseRotation = newRot;

    QString msg = QString::number((rotationZ));
    ui->rotationEditZ->setText(msg);
}

void MainWindow::changeTransformTranslateAmountX(int) 
{
	int incX = ui->translationAmountX->value() - lastTx;
	lastTx = ui->translationAmountX->value() ;
        ((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->baseTranslation.x() += incX;
    QString msg = QString::number(ui->translationAmountX->value());
	ui->translationEditX->setText(msg);

}

void MainWindow::changeTransformTranslateAmountY(int) 
{
	int incY = ui->translationAmountY->value() - lastTy;
	lastTy = ui->translationAmountY->value() ;
        ((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->baseTranslation.y() += incY;

    QString msg = QString::number(ui->translationAmountY->value());
    ui->translationEditY->setText(msg);
}

void MainWindow::changeTransformTranslateAmountZ(int) {
  	int incZ = ui->translationAmountZ->value() - lastTz;
	lastTz = ui->translationAmountZ->value() ;
        ((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->baseTranslation.z() += incZ;

    QString msg = QString::number(ui->translationAmountZ->value());
    ui->translationEditZ->setText(msg);
}

MainWindow::~MainWindow()
{

}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    int current = 0;
    bool checked = false;

	SolverManager* manager = ((BeanViewer*)(ui->glCustomWidget))->solverManager;
	SolverVerlet* verlet;
	if (manager->solvers[0].size() > 0) {
		verlet = (SolverVerlet*) (manager->solvers[0][manager->solvers[0].size()-1]);
	}
	double totalTension = 0;

    switch(event->key())
    {
    case Qt::Key_Up:
        current = ui->SliceSelectorXZ->value();
        if(current + 5 <= ui->SliceSelectorXZ->maximum())
        {
            ui->SliceSelectorXZ->setValue(current+5);
            // TOFIX
            //ui->glCustomWidget->ChangeSliceXZ(current+5);
        }
        break;
    case Qt::Key_Down:
        current = ui->SliceSelectorXZ->value();
        if(current - 5 >= ui->SliceSelectorXZ->minimum())
        {
            ui->SliceSelectorXZ->setValue(current-5);
            // TOFIX
            //ui->glCustomWidget->ChangeSliceXZ(current-5);
        }
        break;
    case Qt::Key_Right:
        current = ui->SliceSelectorXY->value();
        if(current + 5 <= ui->SliceSelectorXY->maximum())
        {
            ui->SliceSelectorXY->setValue(current+5);
            // TOFIX
            //ui->glCustomWidget->ChangeSliceXY(current+5);
        }
        break;
    case Qt::Key_Left:
        current = ui->SliceSelectorXY->value();
        if(current - 5 >= ui->SliceSelectorXY->minimum())
        {
            ui->SliceSelectorXY->setValue(current-5);
            // TOFIX
            //ui->glCustomWidget->ChangeSliceXY(current-5);
        }
        break;

    case Qt::Key_Plus:
        current = ui->DistancesVertSource->value();
        ui->DistancesVertSource->setValue(current+1);
        distancesSourceValueChange(current+1);
        break;

    case Qt::Key_Minus:
        current = ui->DistancesVertSource->value();
        if(current >=1)
        {
            ui->DistancesVertSource->setValue(current-1);
            distancesSourceValueChange(current-1);
        }
        break;

    case Qt::Key_0:
        checked = ui->visibility_btn->isChecked();
        toogleVisibility(!checked);
        ui->visibility_btn->setChecked(!checked);
        break;
    case Qt::Key_1:
        DataVisualizationChange(VIS_LABELS);
        ui->DataVisualizationCombo->setCurrentIndex(VIS_LABELS);
        break;
    case Qt::Key_2:
        DataVisualizationChange(VIS_SEGMENTATION);
        ui->DataVisualizationCombo->setCurrentIndex(VIS_SEGMENTATION);
        break;
    case Qt::Key_3:
        DataVisualizationChange(VIS_BONES_SEGMENTATION);
        ui->DataVisualizationCombo->setCurrentIndex(VIS_BONES_SEGMENTATION);
        break;
    case Qt::Key_4:
        DataVisualizationChange(VIS_SEG_PASS);
        ui->DataVisualizationCombo->setCurrentIndex(VIS_SEG_PASS);
        break;
    case Qt::Key_5:
        DataVisualizationChange(VIS_WEIGHTS);
        ui->DataVisualizationCombo->setCurrentIndex(VIS_WEIGHTS);
        break;
    case Qt::Key_6:
        ShadingModeChange(0);
        ui->shadingModeSelection->setCurrentIndex(0);
        ui->infoData->setText("Smooth shading mode");
        break;
    case Qt::Key_7:
        ShadingModeChange(1);
        ui->shadingModeSelection->setCurrentIndex(1);
        ui->infoData->setText("Flat shading Mode");
        break;
    case Qt::Key_8:
        ShadingModeChange(2);
        ui->shadingModeSelection->setCurrentIndex(2);
        ui->infoData->setText("Blend shading mode");
        break;
    case Qt::Key_9:
		verlet->printStuff = true;

        break;
	case Qt::Key_C:
		
    default:
        break;
    }
}
