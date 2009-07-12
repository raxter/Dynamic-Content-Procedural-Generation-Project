#include "display.h"

#include <QDebug>

namespace ProcGen {

namespace AbstractGameComponent {

  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
Display::Display()
{
}

  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
Display::~Display()
{
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Display::requestReady() {
  qDebug() << "Display::requestReady";
  emit ready(*this);
}

  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Display::initRenderStep()
{
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Display::cleanupRenderStep()
{
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Display::drawCube(double cx, double cy, double cz, double sx, double sy, double sz) const
{
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Display::drawPolygon(const QVector<double>& points) const
{
}

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */

