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

	connect(ui->lookRadius, SIGNAL(valueChanged(int)), this, SLOT(changeLookPointRadius(int)));

	connect(ui->verletRigidness, SIGNAL(toggled(bool)), this, SLOT(toggleVerletRigidness(bool)));

	lastX = lastY = lastZ = 0;
	lastLX = lastLY = lastLZ = 0;
    
}

void MainWindow::toggleVerletRigidness(bool) {
	((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->rigidness = ui->verletRigidness->isChecked();
}

void MainWindow::changeLookPointRadius(int) {
	int newRadius = ui->lookRadius->value();
	int selectedSnake = ui->snakeSelector->currentIndex();
	if (selectedSnake == 0) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->brains.size(); ++i) {
			Intelligence* brain = ((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[i];
			brain->globalLookPointRadius = newRadius;
			brain->globalThita = brain->fRand(0, M_PI/6.0);
			brain->globalPhi = brain->fRand(0, 2.0*M_PI);
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
		((BeanViewer*)ui->glCustomWidget)->solverManager->brains[i]->setState(index);
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
	/*for (int sk = 0; sk < ui->glCustomWidget->escena->skeletons.size(); ++sk) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->idealChains[0]->positions.size(); ++i) {
			((BeanViewer*)(ui->glCustomWidget))->solverManager->verlets[sk]->lookPoint += Vector3d(increment/10.0,0,0);
		}
	}*/
	
	if (index == 0) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->brains.size(); ++i) {
			((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[i]->globalLookPoint.x() += increment/10.0;
		}
	} else {
		((BeanViewer*)(ui->glCustomWidget))->solverManager->brains[index-1]->lookPoint.x() += increment/10.0;
	}
		
	//((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->lookPoint += Vector3d(increment/10.0, 0, 0);
}

void MainWindow::changeLookY(int) {
	int increment = ui->lookY->value() - lastLY;
	lastLY = ui->lookY->value();
	int index = ui->snakeSelector->currentIndex();
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

        ((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->baseTranslation.x() = ui->translationAmountX->value();
    QString msg = QString::number(ui->translationAmountX->value());
	ui->translationEditX->setText(msg);

}

void MainWindow::changeTransformTranslateAmountY(int) 
{

        ((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->baseTranslation.y() = ui->translationAmountY->value();

    QString msg = QString::number(ui->translationAmountY->value());
    ui->translationEditY->setText(msg);
}

void MainWindow::changeTransformTranslateAmountZ(int) {
  
        ((BeanViewer*)(ui->glCustomWidget))->solverManager->solverData->baseTranslation.z() = ui->translationAmountZ->value();

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
        //ShadingModeChange(3);
        //ui->shadingModeSelection->setCurrentIndex(3);
        //ui->infoData->setText("Lines shading mode");
		//((BeanViewer*) ui->glCustomWidget)->solverManager->dumpVectors = !((BeanViewer*) ui->glCustomWidget)->solverManager->dumpVectors;
        break;
	case Qt::Key_C:
		
    default:
        break;
    }
}
