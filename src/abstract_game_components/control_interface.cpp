#include "control_interface.h"

#include <QDebug>

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
void ControlInterface::requestReady() {
  qDebug() << "ControlInterface::requestReady";
  emit ready(*this);
}

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */
