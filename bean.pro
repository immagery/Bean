#-------------------------------------------------
#
# Project created by QtCreator 2013-10-02T13:03:31
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets xml

TARGET = beanViewer
TEMPLATE = app

config += CONSOLE
CONFIG += static

win32{
    #DEBUG
    #LIBS += ../lib/QGLViewerd2.lib
    #RELEASE
    #LIBS += ../lib/QGLViewer2.lib
}

LIBS += ../lib/QGLViewerd2.lib

INCLUDEPATH += ../adri \
                ../adri/gui \
                ../adri/DataStructures \
                ../adri/render \
                ../adri/utils \
                ../include \
                ../thirdparty/vcglib \
                ../thirdparty/libQGLViewer-2.4.0


SOURCES += main.cpp\
        mainwindow.cpp \
    ../adri/DataStructures/SurfaceData.cpp \
    ../adri/DataStructures/skeleton.cpp \
    ../adri/DataStructures/Scene.cpp \
    ../adri/DataStructures/Object.cpp \
    ../adri/DataStructures/Node.cpp \
    ../adri/DataStructures/Modelo.cpp \
    ../adri/DataStructures/InteriorDistancesData.cpp \
    ../adri/DataStructures/grid3D.cpp \
    ../adri/DataStructures/Geometry.cpp \
    ../adri/DataStructures/Deformer.cpp \
    ../adri/DataStructures/DataStructures.cpp \
    ../adri/DataStructures/Cage.cpp \
    ../adri/DataStructures/AnimationManager.cpp \
    ../adri/DataStructures/Animation.cpp \
    ../adri/gui/treemodel.cpp \
    ../adri/gui/treeitem.cpp \
    ../adri/gui/selectionManager.cpp \
    ../adri/gui/outliner.cpp \
    ../adri/gui/object.cpp \
    ../adri/gui/manipulatedFrameSetConstraint.cpp \
    ../adri/gui/KeyframeBar.cpp \
    ../adri/gui/guicon.cpp \
    ../adri/gui/DrawObject.cpp \
    ../adri/gui/bar.cpp \
    ../adri/gui/AdriViewer.cpp \
    ../adri/render/skeletonRender.cpp \
    ../adri/render/shadingNode.cpp \
    ../adri/render/modeloRender.cpp \
    ../adri/render/gridRender.cpp \
    ../adri/render/GeometryRender.cpp \
    ../adri/render/clipingPlaneRender.cpp \
    ../adri/utils/UtilQT.cpp \
    ../adri/utils/util.cpp \
    ../adri/utils/ioWeights.cpp \
    ../adri/utils/ioVCG.cpp \
    BeanViewer.cpp

HEADERS  += mainwindow.h \
    ../adri/DataStructures/SurfaceData.h \
    ../adri/DataStructures/skeleton.h \
    ../adri/DataStructures/Scene.h \
    ../adri/DataStructures/Object.h \
    ../adri/DataStructures/Node.h \
    ../adri/DataStructures/Modelo.h \
    ../adri/DataStructures/InteriorDistancesData.h \
    ../adri/DataStructures/grid3D.h \
    ../adri/DataStructures/Geometry.h \
    ../adri/DataStructures/Deformer.h \
    ../adri/DataStructures/DefNode.h \
    ../adri/DataStructures/DataStructures.h \
    ../adri/DataStructures/Cage.h \
    ../adri/DataStructures/AnimationManager.h \
    ../adri/DataStructures/Animation.h \
    ../adri/gui/treemodel.h \
    ../adri/gui/treeitem.h \
    ../adri/gui/SelectionManager.h \
    ../adri/gui/outliner.h \
    ../adri/gui/object.h \
    ../adri/gui/manipulatedFrameSetConstraint.h \
    ../adri/gui/KeyframeBar.h \
    ../adri/gui/guicon.h \
    ../adri/gui/DrawObject.h \
    ../adri/gui/bar.h \
    ../adri/gui/AdriViewer.h \
    ../adri/render/skeletonRender.h \
    ../adri/render/shadingNode.h \
    ../adri/render/modeloRender.h \
    ../adri/render/gridRender.h \
    ../adri/render/GeometryRender.h \
    ../adri/render/clipingPlaneRender.h \
    ../adri/utils/UtilQT.h \
    ../adri/utils/utilGL.h \
    ../adri/utils/util.h \
    ../adri/utils/ioWeights.h \
    BeanViewer.h

FORMS    += mainwindow.ui
