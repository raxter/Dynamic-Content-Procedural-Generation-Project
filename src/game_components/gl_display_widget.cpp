#include "gl_display.h"

#include <QDebug>
#include <QMouseEvent>
#include <QKeyEvent>

namespace ProcGen {

namespace GameComponent {

  
/****************************************************************************
**
** Author: Richard Baxter
**
** Default Constructor
**
****************************************************************************/
GLDisplayWidget::GLDisplayWidget(QWidget* parent) : QGLWidget(parent)
{
  setMouseTracking ( true );
  grabKeyboard ();
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
GLDisplayWidget::~GLDisplayWidget()
{
  releaseKeyboard ();
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::initializeGL()
{
  qDebug() << "initializeGL";         
  
  glClearColor(0.0, 0.0, 0.0, 0.0);

  /*FIXME this should move, similar vibe to rendering... if I ever get that working*/
  glEnable(GL_DEPTH_TEST);
  
  glMatrixMode(GL_MODELVIEW);

  qDebug() << "currentContext: " << QGLContext::currentContext ();

}



/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::paintGL() {
  emit sendingContext();
}


void GLDisplayWidget::resizeGL ( int width, int height ) {
  qDebug() << "resizeGL - " << width << " " << height;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, width, height);
  //glOrtho(	0, width, height, 0, -10000, 10000);
  gluPerspective( 75/*GLdouble	fovy*/,
			            1/*GLdouble	aspect*/,
			            0.1/*GLdouble	zNear*/,
			            10000/*GLdouble	zFar*/ );

  glMatrixMode(GL_MODELVIEW);
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::mouseMoveEvent ( QMouseEvent * event ) {
  emit mouseMoved(event->pos ());
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::wheelEvent ( QWheelEvent * event ) {

}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::mousePressEvent ( QMouseEvent * event ) {
  emit mouseEvent(event->button (), true);
}
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::mouseReleaseEvent ( QMouseEvent * event ) {
  emit mouseEvent(event->button (), false);
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::keyPressEvent ( QKeyEvent * event ) {
  //qDebug() << "keyPressEvent";
  if (!event->isAutoRepeat () )
    emit keyEvent(event->key (), true);
}
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::keyReleaseEvent ( QKeyEvent * event ) {
  //qDebug() << "keyReleaseEvent";
  if (!event->isAutoRepeat ())
    emit keyEvent(event->key (), false);
}
  
} /* end of namespace GameComponent */

} /* end of namespace ProcGen */













