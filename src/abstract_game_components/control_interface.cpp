#include "control_interface.h"

#include <QDebug>
#include <QThread>
#include <QGLContext>

namespace ProcGen {

namespace AbstractGameComponent {

  

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
ControlInterface::ControlInterface() {

}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
ControlInterface::~ControlInterface() {


}

  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void ControlInterface::incomingMousePosition(const QPoint& position) {
  incomingVarMutex.lock();
  currentMousePosition = position;
  incomingVarMutex.unlock();
  //qDebug() << "ControlInterface::incomingMousePosition - " << position;
}

  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void ControlInterface::incomingKeyEvent(int keyCode, bool isKeyDown) {
  qDebug() << "ControlInterface::incomingKeyEvent - " << keyCode << ": " << (isKeyDown?"pressed ":"released");
  incomingVarMutex.lock();
  if (isKeyDown) {
    keysDown.append(keyCode);
  }
  else {
    keysUp.append(keyCode);
  } 
  incomingVarMutex.unlock();
}

  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void ControlInterface::incomingMouseButtonEvent(int mouseCode, bool isMouseButtonDown) {
  qDebug() << "ControlInterface::incomingMouseButtonEvent - " << mouseCode << ": " << (isMouseButtonDown?"pressed ":"released");
  incomingVarMutex.lock();
  if (isMouseButtonDown) {
    mouseButtonsDown.append(mouseCode);
  }
  else {
    mouseButtonsUp.append(mouseCode);
  } 
  incomingVarMutex.unlock();
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void ControlInterface::eventStep() {
  //qDebug() << "ControlInterface::eventStep" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();

  incomingVarMutex.lock();
  
  /* mouse movement events */
  mousePosition = currentMousePosition;
  mouseMovement = currentMousePosition - oldMousePosition;
  
  oldMousePosition = mousePosition;
  
  /* mouse events */
  
  QList<int> forceMouseUp, mouseJustDownSet;
  
  Q_FOREACH(int i, forceMouseTapUp) {
    mouseTapped[i] = false;
  }
  forceMouseTapUp.clear();
  
  Q_FOREACH(int i, mouseButtonsDown) {
    mouseDown[i] = true;
    mouseTapped[i] = true;
    mouseJustDown[i] = true;
    mouseJustDownSet.append(i);
    forceMouseTapUp.append(i);
  }
  mouseButtonsDown.clear();
  
  Q_FOREACH(int i, mouseButtonsUp) {
    if (mouseJustDown[i]) {
      forceMouseUp.append(i);
    }
    else {
      mouseDown[i] = false;
    }
  }
  mouseButtonsUp.clear();
  
  Q_FOREACH(int i, forceMouseUp) {
    mouseButtonsUp.append(i);
  }
  
  
  Q_FOREACH(int i, mouseJustDownSet) {
    mouseJustDown[i] = false;
  }
  
  /* key events */
  
  QList<int> forceKeyUp, keyJustDownSet;
  
  Q_FOREACH(int i, forceTapUp) {
    keyTapped[i] = false;
  }
  forceTapUp.clear();
  
  Q_FOREACH(int i, keysDown) {
    keyDown[i] = true;
    keyTapped[i] = true;
    keyJustDown[i] = true;
    keyJustDownSet.append(i);
    forceTapUp.append(i);
  }
  keysDown.clear();
  
  Q_FOREACH(int i, keysUp) {
    if (keyJustDown[i]) {
      forceKeyUp.append(i);
    }
    else {
      keyDown[i] = false;
    }
  }
  keysUp.clear();
  
  Q_FOREACH(int i, forceKeyUp) {
    keysUp.append(i);
  }
  
  
  Q_FOREACH(int i, keyJustDownSet) {
    keyJustDown[i] = false;
  }
  
  incomingVarMutex.unlock();
  //qDebug() << "END ControlInterface::eventStep" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
QPoint ControlInterface::getMousePosition() const {
  return mousePosition;

}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
QPoint ControlInterface::getMouseMovement() const {
  return mouseMovement;

}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
bool ControlInterface::isMouseDown(int mouseButtonCode) const {
  //qDebug() << mouseButtonCode << "down? => " << mouseDown[mouseButtonCode];
  return mouseDown[mouseButtonCode];
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
bool ControlInterface::isMouseTapped(int mouseButtonCode) const {
  return mouseTapped[mouseButtonCode];
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
bool ControlInterface::isKeyDown(int keyCode) const {
  //qDebug() << keyCode << "down? => " << keyDown[keyCode];
  return keyDown[keyCode];
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
bool ControlInterface::isKeyTapped(int keyCode) const {
  return keyTapped[keyCode];
}

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */
