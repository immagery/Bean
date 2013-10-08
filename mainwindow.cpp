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

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    AdriMainWindow(parent)
    //ui(new Ui::MainWindow)
{

    ui->glCustomWidget->parent = this;

    // Transform
    connect(ui->rotationAmountX, SIGNAL(valueChanged(int)), this, SLOT(changeTransformRotateAmountX(int)));
    connect(ui->rotationAmountY, SIGNAL(valueChanged(int)), this, SLOT(changeTransformRotateAmountY(int)));
    connect(ui->rotationAmountZ, SIGNAL(valueChanged(int)), this, SLOT(changeTransformRotateAmountZ(int)));
    connect(ui->resetRotation, SIGNAL(clicked()), this, SLOT(resetRotationValues()));

    // Animation
    connect(ui->addKeyframe, SIGNAL(clicked()), this, SLOT(addAnimationKeyframe()));
    connect(ui->toggleAnim, SIGNAL(clicked()), this, SLOT(toggleAnimation()));
    connect(ui->animSlider, SIGNAL(valueChanged(int)), this, SLOT(changeFrame(int)));
    connect(ui->glCustomWidget, SIGNAL(changedFrame(int)), this, SLOT(changeAnimSlider(int)));
	connect(ui->saveAnim, SIGNAL(clicked()), this, SLOT(saveAnimation()));
	connect(ui->loadAnim, SIGNAL(clicked()), this, SLOT(loadAnimation()));

	// Simulation
	connect(ui->toggleSim, SIGNAL(clicked()), this, SLOT(toggleSimulation()));
}

void MainWindow::changeTransformRotateAmountX(int) {

    object *selectedObject = NULL;
    if (ui->glCustomWidget->selMgr.selection.size() > 0)
        selectedObject = ui->glCustomWidget->selMgr.selection.back();

    if (selectedObject != NULL)
        selectedObject->pos.X() = ui->rotationAmountX->value()/10.0;

	ui->glCustomWidget->particles->xvalue = ui->rotationAmountX->value()/10.0;

    QString msg = QString::number(ui->rotationAmountX->value());
    ui->rotationEditX->setText(msg);

}

void MainWindow::changeTransformRotateAmountY(int) {
    object *selectedObject = NULL;
    if (ui->glCustomWidget->selMgr.selection.size() > 0)
        selectedObject = ui->glCustomWidget->selMgr.selection.back();

    if (selectedObject != NULL)
        //selectedObject->rot.Y() = ui->rotationAmountY->value();

    QString msg = QString::number(ui->rotationAmountY->value());
    //ui->rotationEditY->setText(msg);


}

void MainWindow::changeTransformRotateAmountZ(int) {
    object *selectedObject = NULL;
    if (ui->glCustomWidget->selMgr.selection.size() > 0)
        selectedObject = ui->glCustomWidget->selMgr.selection.back();

    if (selectedObject != NULL)
        //selectedObject->rot.Z() = ui->rotationAmountZ->value();

    QString msg = QString::number(ui->rotationAmountZ->value());
    //ui->rotationEditZ->setText(msg);

}

void MainWindow::resetRotationValues() {
    object *selectedObject = NULL;

    if (ui->glCustomWidget->selMgr.selection.size() > 0)
        selectedObject = ui->glCustomWidget->selMgr.selection.back();


    //if (selectedObject != NULL) selectedObject->rot.SetZero();
    ui->rotationAmountX->setValue(0);
    ui->rotationAmountY->setValue(0);
    ui->rotationAmountZ->setValue(0);

}

void MainWindow::addAnimationKeyframe() {


    object *selectedObject = NULL;
    if (ui->glCustomWidget->selMgr.selection.size() > 0)
        selectedObject = ui->glCustomWidget->selMgr.selection.back();

    if (selectedObject == NULL) return;

    int id = selectedObject->nodeId;
    int frame = ui->animSlider->value();
    AdriViewer * viewer = ui->glCustomWidget;
    if (!viewer->aniManager.objectHasAnimation(id)) viewer->aniManager.addAnimation(id);
    viewer->aniManager.addKeyFrame(id, frame, 0, 0, 0,
                                ui->rotationAmountX->value(),
                                ui->rotationAmountY->value(),
                                ui->rotationAmountZ->value(),
                                0, 0, 0);

	ui->kfBar->addKeyframe(frame, 150);
}

void MainWindow::toggleAnimation() {
    ui->glCustomWidget->aniManager.animationEnabled = !ui->glCustomWidget->aniManager.animationEnabled;
	if (ui->glCustomWidget->aniManager.animationEnabled) ui->toggleSim->setEnabled(false);
	else ui->toggleSim->setEnabled(true);
}

void MainWindow::toggleSimulation() {
    ui->glCustomWidget->aniManager.simulationEnabled = !ui->glCustomWidget->aniManager.simulationEnabled;
	if (ui->glCustomWidget->aniManager.simulationEnabled) ui->toggleAnim->setEnabled(false);
	else ui->toggleAnim->setEnabled(true);
}

void MainWindow::changeFrame(int frame) {
    // Change the frame label to its correct value
    QString msg = QString::number(frame);
    ui->frameLabel->setText(msg);

    // Tell animManager to load that frame info
    ui->glCustomWidget->frame = frame;

    // Update UI values accordingly
    for (unsigned int i = 0; i < ui->glCustomWidget->escena->skeletons.size(); ++i) {
        skeleton* skt = ((skeleton*) ui->glCustomWidget->escena->skeletons[i]);
        for (unsigned int j = 0; j < skt->joints.size(); ++j) {
            if (ui->glCustomWidget->aniManager.objectHasAnimation(skt->joints[j]->nodeId)) {
                object * joint = (object *)skt->joints[j];
                //joint->rot = ui->glCustomWidget->aniManager.getRotation(skt->joints[j]->nodeId, frame);

                ui->rotationAmountX->blockSignals(true);
                //ui->rotationAmountX->setValue(joint->rot.X());
                //ui->rotationEditX->setText(QString("%1").arg(joint->rot.X(), 3, 'g', 3));
                ui->rotationAmountX->blockSignals(false);

                ui->rotationAmountY->blockSignals(true);
                //ui->rotationAmountY->setValue(joint->rot.Y());
                //ui->rotationEditY->setText(QString("%1").arg(joint->rot.Y(), 3, 'g', 3));
                ui->rotationAmountY->blockSignals(false);

                ui->rotationAmountZ->blockSignals(true);
                //ui->rotationAmountZ->setValue(joint->rot.Z());
                //ui->rotationEditZ->setText(QString("%1").arg(joint->rot.Z(), 3, 'g', 3));
                ui->rotationAmountZ->blockSignals(false);

            }
        }
    }
}

void MainWindow::changeAnimSlider(int val) {
    ui->animSlider->setValue(val);
}

void MainWindow::saveAnimation() {
	string path = ui->glCustomWidget->sPathGlobal.toStdString();
	ui->glCustomWidget->aniManager.saveAnimation("../models/animation.txt", ui->glCustomWidget->escena);
}

void MainWindow::loadAnimation() {
	QFileDialog inFileDialog(0, "Selecciona un fichero", ui->glCustomWidget->sPathGlobal, "*.off *.txt");
	inFileDialog.setFileMode(QFileDialog::ExistingFile);
	    QStringList fileNames;
     if (inFileDialog.exec())
         fileNames = inFileDialog.selectedFiles();

    if(fileNames.size() == 0)
        return;

    QFileInfo sPathAux(fileNames[0]);
    QString aux = sPathAux.canonicalPath();

	ui->glCustomWidget->aniManager.loadAnimations(fileNames[0].toStdString(), ui->glCustomWidget->escena);
	ui->kfBar->addListOfKeyframes(ui->glCustomWidget->aniManager.getFrames(), 150);
	ui->kfBar->repaint();
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
        ShadingModeChange(3);
        ui->shadingModeSelection->setCurrentIndex(3);
        ui->infoData->setText("Lines shading mode");
        break;
    default:
        break;
    }
}
