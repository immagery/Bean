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
	connect(ui->useVerlet, SIGNAL(toggled(bool)), this, SLOT(toggleVerlet(bool)));

	/* TODO: se habrá chafado al fusionar la interficie... ya se ve que la interficie cuesta
	connect(ui->verletStiffness, SIGNAL(valueChanged(int)), this, SLOT(changeVerletStiffness(int)));

	connect(ui->verletX, SIGNAL(valueChanged(int)), this, SLOT(changeVerletX(int)));
    connect(ui->verletY, SIGNAL(valueChanged(int)), this, SLOT(changeVerletY(int)));
    connect(ui->verletZ, SIGNAL(valueChanged(int)), this, SLOT(changeVerletZ(int)));

	connect(ui->lookX, SIGNAL(valueChanged(int)), this, SLOT(changeLookX(int)));
    connect(ui->lookY, SIGNAL(valueChanged(int)), this, SLOT(changeLookY(int)));
    connect(ui->lookZ, SIGNAL(valueChanged(int)), this, SLOT(changeLookZ(int)));
	*/

	lastX = lastY = lastZ = 0;
	lastLX = lastLY = lastLZ = 0;
    
}


/* TODO: interface_merge
void MainWindow::changeVerletStiffness(int) {
	//((BeanViewer*)ui->glCustomWidget)->solverManager->verlet->velocityDamping = ui->velDamp->value() / 100.0;
	for (int i = 0; i < ui->glCustomWidget->escena->skeletons.size(); ++i) {
		((BeanViewer*)ui->glCustomWidget)->solverManager->verlets[i]->stiffness = ui->verletStiffness->value() / 10000.0;
	}
}
*/

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
	ui->useVerlet->setCheckable(true);
	ui->useVerlet->toggle();
	ui->useSolvers->setCheckable(true);
}

void MainWindow::toggleVerlet(bool) {
	bool b = ui->useVerlet->isChecked();
	//((BeanViewer*)(ui->glCustomWidget))->solverManager->hasVerlet = b;
}

void MainWindow::toggleSolvers(bool) {
	bool b = ui->useVerlet->isChecked();
	//((BeanViewer*)(ui->glCustomWidget))->solverManager->oscillation = b;
}

/* TODO: se debe haber eliminado de la interficie.
void MainWindow::changeVerletX(int) {
	int increment = ui->verletX->value() - lastX;
	lastX = ui->verletX->value();
	for (int sk = 0; sk < ui->glCustomWidget->escena->skeletons.size(); ++sk) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->idealChains[0]->positions.size(); ++i) {
			((BeanViewer*)(ui->glCustomWidget))->solverManager->idealChains[sk]->positions[i].x() += increment;
		}
	}
}

void MainWindow::changeVerletY(int) {
	int increment = ui->verletY->value() - lastY;
	lastY = ui->verletY->value();
	for (int sk = 0; sk < ui->glCustomWidget->escena->skeletons.size(); ++sk) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->idealChains[0]->positions.size(); ++i) {
			((BeanViewer*)(ui->glCustomWidget))->solverManager->idealChains[sk]->positions[i].y() += increment;
		}
	}
}

void MainWindow::changeVerletZ(int) {
	int increment = ui->verletZ->value() - lastZ;
	lastZ = ui->verletZ->value();
	for (int sk = 0; sk < ui->glCustomWidget->escena->skeletons.size(); ++sk) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->idealChains[0]->positions.size(); ++i) {
			((BeanViewer*)(ui->glCustomWidget))->solverManager->idealChains[sk]->positions[i].z() += increment;
		}
	}
}

void MainWindow::changeLookX(int) {
	int increment = ui->lookX->value() - lastLX;
	lastLX = ui->lookX->value();
	for (int sk = 0; sk < ui->glCustomWidget->escena->skeletons.size(); ++sk) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->idealChains[0]->positions.size(); ++i) {
			((BeanViewer*)(ui->glCustomWidget))->solverManager->verlets[sk]->lookPoint += Vector3d(increment/10.0,0,0);
		}
	}
}

void MainWindow::changeLookY(int) {
	int increment = ui->lookY->value() - lastLY;
	lastLY = ui->lookY->value();
	for (int sk = 0; sk < ui->glCustomWidget->escena->skeletons.size(); ++sk) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->idealChains[0]->positions.size(); ++i) {
			((BeanViewer*)(ui->glCustomWidget))->solverManager->verlets[sk]->lookPoint += Vector3d(0,increment/10.0,0);
		}
	}
}

void MainWindow::changeLookZ(int) {
	int increment = ui->lookZ->value() - lastLZ;
	lastLZ = ui->lookZ->value();
	for (int sk = 0; sk < ui->glCustomWidget->escena->skeletons.size(); ++sk) {
		for (int i = 0; i < ((BeanViewer*)(ui->glCustomWidget))->solverManager->idealChains[0]->positions.size(); ++i) {
			((BeanViewer*)(ui->glCustomWidget))->solverManager->verlets[sk]->lookPoint += Vector3d(0,0,increment/10.0);
		}
	}
}
*/

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
		((BeanViewer*) ui->glCustomWidget)->solverManager->dumpVectors = !((BeanViewer*) ui->glCustomWidget)->solverManager->dumpVectors;
		break;
        break;
	case Qt::Key_C:
		
    default:
        break;
    }
}
