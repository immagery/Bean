#ifndef BEANVIEWER_H
#define BEANVIEWER_H

#include "AdriViewer.h"

class BeanViewer : public AdriViewer
{
        Q_OBJECT
public:
    BeanViewer(QWidget * parent = 0, const QGLWidget * shareWidget = 0, Qt::WindowFlags flags = 0);

protected:
	virtual void draw();
};

#endif // BEANVIEWER_H
