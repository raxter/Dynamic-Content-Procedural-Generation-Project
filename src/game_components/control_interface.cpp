#include "control_interface.h"

#include <QDebug>

namespace ProcGen {

namespace GameComponent {

  

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
ControlInterface::ControlInterface(const QWidget * eventSender) : AbstractGameComponent::ControlInterface() {

  connect( eventSender, SIGNAL(mouseMoved(const QPoint&)), this, SLOT(incomingMousePosition(const QPoint&)) );

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
  newMousePosition = position;
  qDebug() << "ControlInterface::incomingMousePosition - " << position;
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void ControlInterface::eventStep() {

  mousePosition = newMousePosition;
  
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
bool ControlInterface::isKeyDown(int keyCode) const {


}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
bool ControlInterface::isKeyTapped(int keyCode) const {


}


} /* end of namespace GameComponent */

} /* end of namespace ProcGen */
