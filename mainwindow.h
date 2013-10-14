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

public slots:



protected:
    virtual void keyPressEvent(QKeyEvent* event);
};

#endif // MAINWINDOW_H
