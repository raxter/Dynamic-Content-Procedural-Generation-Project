#include "gl_display.h"

#include <QDebug>
#include <QThread>

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
  
  connect(this, SIGNAL (pleaseDraw()), this, SLOT(updateGL()));
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
  QMutexLocker locker(&GLMutex);
  QMutexLocker initLocker(&initMutex);
  //qDebug() << "GLDisplayWidget::initializeGL" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();  
  if (!isInitialized)  {
    
    displayer.initialize();
    gameCore.initStep(displayer);
    isInitialized = true;
    //emit initializeComplete();
  }
  //qDebug() << "END GLDisplayWidget::initializeGL" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();  
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
bool GLDisplayWidget::initialized()
{
  QMutexLocker locker(&GLMutex);
  QMutexLocker initLocker(&initMutex);
  //qDebug() << "GLDisplayWidget::initialized" << " Thread: " << QThread::currentThread ();
  //qDebug() << "END GLDisplayWidget::initialized" << " Thread: " << QThread::currentThread ();
  return isInitialized;
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::initStep()
{
  QMutexLocker locker(&GLMutex);
  qDebug() << "GLDisplayWidget::initStep" << " Thread: " << QThread::currentThread ();
  glInit ();
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::logicStep()
{
  QMutexLocker locker(&GLMutex);
  //qDebug() << "GLDisplayWidget::logicStep" << " Thread: " << QThread::currentThread ();
  
  controlInterface.eventStep();
  gameCore.logicStep(controlInterface);
  
  //qDebug() << "END GLDisplayWidget::logicStep" << " Thread: " << QThread::currentThread ();
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::renderStep()
{
  QMutexLocker locker(&GLMutex);
  //qDebug() << "GLDisplayWidget::renderStep" << " Thread: " << QThread::currentThread ();
  locker.unlock();
  //glDraw ();
  //updateGL();
  emit pleaseDraw();
  //qDebug() << "END GLDisplayWidget::renderStep" << " Thread: " << QThread::currentThread ();
}
  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::cleanUpStep()
{
  QMutexLocker locker(&GLMutex);
  //qDebug() << "GLDisplayWidget::cleanUpStep" << " Thread: " << QThread::currentThread ();

}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::paintGL() {
  QMutexLocker locker(&GLMutex);
  //qDebug() << "GLDisplayWidget::paintGL" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();  
  makeCurrent ();
  QMutexLocker resizeLocker(&resizeMutex);
  if (QGLContext::currentContext ()) {
    //qDebug() << "GLContext not NULL";
    if (performResize) {
      displayer.resize(resize_width, resize_height);
      gameCore.resizeStep(resize_width, resize_height);
      performResize = false;
    }
    gameCore.renderStep(displayer);
  }
  else {
    //qDebug() << "GLContext NULL!!";
    }
  //qDebug() << "END GLDisplayWidget::paintGL" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();  
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::resizeGL ( int width, int height ) {
  QMutexLocker locker(&GLMutex);
  //qDebug() << "GLDisplayWidget::resizeGL - " << width << " " << height << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
  QMutexLocker resizeLocker(&resizeMutex);
  performResize = true;
  resize_width = width;
  resize_height = height;
  //qDebug() << "END GLDisplayWidget::resizeGL - " << width << " " << height << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
//void GLDisplayWidget::paintEvent ( QPaintEvent * event ) {
//  qDebug() << "GLDisplayWidget::paintEvent" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
//}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
/*void GLDisplayWidget::mouseMoveEvent ( QMouseEvent * event ) {
  emit mouseMoved(event->pos ());
}*/

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
/*void GLDisplayWidget::wheelEvent ( QWheelEvent * event ) {

}*

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
/*void GLDisplayWidget::mousePressEvent ( QMouseEvent * event ) {
  emit mouseEvent(event->button (), true);
}*/
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
/*void GLDisplayWidget::mouseReleaseEvent ( QMouseEvent * event ) {
  emit mouseEvent(event->button (), false);
}*/

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::keyPressEvent ( QKeyEvent * event ) {
  qDebug() << "keyPressEvent";
  if (!event->isAutoRepeat () )
    emit keyEvent(event->key (), true);
}
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GLDisplayWidget::keyReleaseEvent ( QKeyEvent * event ) {
  qDebug() << "keyReleaseEvent";
  if (!event->isAutoRepeat ())
    emit keyEvent(event->key (), false);
}
  
} /* end of namespace GameComponent */

} /* end of namespace ProcGen */













