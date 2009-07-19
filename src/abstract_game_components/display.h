#ifndef __PROCGEN_ABSTRACTGAMECOMPONENT_DISPLAY_H__
#define __PROCGEN_ABSTRACTGAMECOMPONENT_DISPLAY_H__

#include <QVector>
#include <QFont>
#include <QObject>
#include <QImage>

#include "Box2D.h"

namespace ProcGen {

namespace AbstractGameComponent {

  
class Display {


  protected: /* class specific */

  Display();
  virtual ~Display();
  
  public: /* methods */
  
  virtual void initialize() = 0;
  
  virtual void resize (int width, int height) = 0;
  
  virtual void initRenderStep() = 0;
  
  virtual unsigned int bindTexture(const QImage& image) const = 0;
  
  virtual void drawBody(b2Body* body) const = 0;
  
  virtual void drawCube(double cx, double cy, double cz, double sx, double sy, double sz) const = 0;
  
  virtual void drawPolygon(const QVector<double>& points) const = 0;
  
  virtual void drawText2D(int x, int y, const QString & str, const QFont & fnt = QFont ( ), int listBase = 2000 ) const = 0;
  //void drawMesh, etc
  
  virtual void cleanupRenderStep() = 0;
  
  
};

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_GAME_ABSTRACTDISPLAY_H__ */

