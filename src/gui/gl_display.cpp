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
GLDisplay::GLDisplay(QWidget* parent) : QGLWidget(parent)
{
  
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
GLDisplay::~GLDisplay()
{

}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::initializeGL()
{
  qDebug() << "initializeGL";
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glEnable(GL_DEPTH_TEST);

}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::updateGL()
{
  qDebug() << "updateGL";
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplay::resizeGL()
{
  qDebug() << "resizeGL";
}
  
  
} /* end of namespace GUI */

} /* end of namespace ProcGen */

