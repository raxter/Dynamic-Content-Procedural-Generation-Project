#ifndef __PROCGEN_GAME_ABSTRACTDISPLAY_H__
#define __PROCGEN_GAME_ABSTRACTDISPLAY_H__

#include <QVector>
#include <QObject>

namespace ProcGen {

namespace Game {

  
class AbstractDisplay : public QObject {

  Q_OBJECT

  protected: /* class specific */

  AbstractDisplay();
  virtual ~AbstractDisplay();
  
  /* Overload these signals!*/
  signals:
  //void displayInitialized();
  void ready();
  
  public slots:
  void requestReady();
  
  public: /* methods */
  
  virtual bool isInitialized() const = 0;
  
  virtual void initRenderStep() = 0;
  
  virtual void drawCube(double cx, double cy, double cz, double sx, double sy, double sz) = 0;
  
  virtual void drawPolygon(const QVector<double>& points) = 0;
  
  //void drawMesh, etc
  
  virtual void cleanupRenderStep() = 0;
  
  
};

} /* end of namespace Game */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_GAME_ABSTRACTDISPLAY_H__ */

