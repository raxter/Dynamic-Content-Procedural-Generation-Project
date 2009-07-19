#include "gl_display.h"

#include <QDebug>

namespace ProcGen {

namespace GameComponent {

  
/****************************************************************************
**
** Author: Richard Baxter
**
** Default Constructor
**
****************************************************************************/
GLDisplayWidget::GLDisplayWidget(AbstractGameComponent::Game& gameCore, AbstractGameComponent::Display& displayer, AbstractGameComponent::ControlInterface& controlInterface, QWidget* parent) : 
            QGLWidget(parent), 
            GameInterface(gameCore),
            displayer(displayer), 
            controlInterface(controlInterface),  
            performResize(false),
            isInitialized(false)
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
  qDebug() << "GLDisplayWidget::initializeGL";      
  qDebug() << "currentContext: " << QGLContext::currentContext ();  
  if (!isInitialized)  {
    displayer.initialize();
    gameCore.initStep(displayer);
    isInitialized = true;
    //emit initializeComplete();
  }
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
bool GLDisplayWidget::initialized()
{
  qDebug() << "GLDisplayWidget::initialized";
  return isInitialized;
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::initStep()
{
  qDebug() << "GLDisplayWidget::initStep";
  glInit ();
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::logicStep()
{
  qDebug() << "GLDisplayWidget::logicStep";
  
  controlInterface.eventStep();
  if (performResize) {
    displayer.resize(resize_width, resize_height);
    gameCore.resizeStep(resize_width, resize_height);
    performResize = false;
  }
  gameCore.logicStep(controlInterface);
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::renderStep()
{
  qDebug() << "GLDisplayWidget::renderStep";
  updateGL ();
}
  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::cleanUpStep()
{
  qDebug() << "GLDisplayWidget::cleanUpStep";

}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::paintGL() {
  qDebug() << "GLDisplayWidget::paintGL";    
  qDebug() << "currentContext: " << QGLContext::currentContext ();    

  gameCore.renderStep(displayer);
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::resizeGL ( int width, int height ) {
  qDebug() << "resizeGL - " << width << " " << height;
  qDebug() << "currentContext: " << QGLContext::currentContext ();
  
  performResize = true;
  resize_width = width;
  resize_height = height;
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













