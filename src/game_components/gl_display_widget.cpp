#include "gl_display.h"

#include <QDebug>

namespace ProcGen {

namespace GUI {

  
/****************************************************************************
**
** Author: Richard Baxter
**
** Default Constructor
**
****************************************************************************/
GLDisplayWidget::GLDisplayWidget(QWidget* parent) : QGLWidget(parent)
{
  
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
GLDisplayWidget::~GLDisplayWidget()
{

}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::initializeGL()
{
  qDebug() << "initializeGL";         
  
  glClearColor(0.0, 1.0, 0.0, 0.0);

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
  qDebug() << "paintGL";
  emit sendingContext();
}


void GLDisplayWidget::resizeGL ( int width, int height ) {
  qDebug() << "resizeGL - " << width << " " << height;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, width, height);
  glOrtho(	0, width, height, 0, -10, 10);
  glMatrixMode(GL_MODELVIEW);
}

  
} /* end of namespace GUI */

} /* end of namespace ProcGen */













