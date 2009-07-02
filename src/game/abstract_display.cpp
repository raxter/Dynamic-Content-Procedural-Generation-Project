#include "abstract_display.h"

#include <QDebug>

namespace ProcGen {

namespace Game {

  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
AbstractDisplay::AbstractDisplay()
{
}

  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
AbstractDisplay::~AbstractDisplay()
{
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void AbstractDisplay::requestReady() {
  qDebug() << "AbstractDisplay::requestReady";
  emit ready();
}

  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void AbstractDisplay::initRenderStep()
{
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void AbstractDisplay::cleanupRenderStep()
{
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void AbstractDisplay::drawCube(double cx, double cy, double cz, double sx, double sy, double sz)
{
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void AbstractDisplay::drawPolygon(const QVector<double>& points)
{
}

} /* end of namespace Game */

} /* end of namespace ProcGen */

