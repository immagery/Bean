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
	void setViewer();

	int lastX, lastY, lastZ;
	int lastLX, lastLY, lastLZ;

public slots:
	void loadSolvers();
	void toggleVerlet(bool);
	void toggleSolvers(bool);
	void changeVerletStiffness(int);
	void changeVerletGravity(int);
	void changeLookX(int);
	void changeLookY(int);
	void changeLookZ(int);
	virtual void changeTransformRotateAmountX(int);
    virtual void changeTransformRotateAmountY(int);
    virtual void changeTransformRotateAmountZ(int);
	virtual void changeTransformTranslateAmountX(int);
    virtual void changeTransformTranslateAmountY(int);
    virtual void changeTransformTranslateAmountZ(int);
	void changedDrawLocators(bool);
	void changeBehaviour(int);
protected:
    virtual void keyPressEvent(QKeyEvent* event);
};

#endif // MAINWINDOW_H
