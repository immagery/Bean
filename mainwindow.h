#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <gui/adrimainwindow.h>
#include <QtCore/QModelIndex>

class TreeModel;
class TreeItem;
class treeNode;

class MainWindow : public AdriMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:

    // Transform
    void changeTransformRotateAmountX(int);
    void changeTransformRotateAmountY(int);
    void changeTransformRotateAmountZ(int);
    void resetRotationValues();

    // Animation
    void addAnimationKeyframe();
    void toggleAnimation();
    void changeFrame(int);
    void changeAnimSlider(int);
	void saveAnimation();
	void loadAnimation();

	// Simulation
	void toggleSimulation();

protected:
    virtual void keyPressEvent(QKeyEvent* event);
};

#endif // MAINWINDOW_H
