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
	ui->glCustomWidget = new BeanViewer(ui->frame);
	ui->glCustomWidget->parent = this;
	setViewer();
	connectSignals();
    
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
