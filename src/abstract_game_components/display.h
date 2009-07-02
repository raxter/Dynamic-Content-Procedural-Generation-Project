#ifndef __PROCGEN_ABSTRACTGAMECOMPONENT_DISPLAY_H__
#define __PROCGEN_ABSTRACTGAMECOMPONENT_DISPLAY_H__

#include <QVector>
#include <QObject>

namespace ProcGen {

namespace AbstractGameComponent {

  
class Display : public QObject {

  Q_OBJECT

  protected: /* class specific */

  Display();
  virtual ~Display();
  
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

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_GAME_ABSTRACTDISPLAY_H__ */

