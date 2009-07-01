#ifndef __PROCGEN_GAME_ABSTRACTDISPLAY_H__
#define __PROCGEN_GAME_ABSTRACTDISPLAY_H__


namespace ProcGen {

namespace Game {

  
class AbstractDisplay {

  public: /* class specific */

  AbstractDisplay();
  virtual ~AbstractDisplay();
  
  public: /* methods */
  
  virtual void initRenderStep();
  
  virtual void drawCube(double cx, double cy, double cz, double sx, double sy, double sz);
  
  virtual void drawPolygon(const QVector<double>& points);
  
  //void drawMesh, etc
  
  virtual void cleanupRenderStep();
  
  
};

} /* end of namespace Game */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_GAME_ABSTRACTDISPLAY_H__ */

