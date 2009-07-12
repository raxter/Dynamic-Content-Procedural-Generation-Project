#ifndef __PROCGEN_ABSTRACTGAMECOMPONENT_DISPLAY_H__
#define __PROCGEN_ABSTRACTGAMECOMPONENT_DISPLAY_H__

#include <QVector>
#include <QFont>
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
  void ready( const Display& );
  
  public slots:
  void requestReady();
  
  public: /* methods */
  
  virtual bool isInitialized() const = 0;
  
  virtual void initRenderStep() = 0;
  
  virtual void drawCube(double cx, double cy, double cz, double sx, double sy, double sz) const = 0;
  
  virtual void drawPolygon(const QVector<double>& points) const = 0;
  
  virtual void drawText2D(int x, int y, const QString & str, const QFont & fnt = QFont ( ), int listBase = 2000 ) const = 0;
  //void drawMesh, etc
  
  virtual void cleanupRenderStep() = 0;
  
  
};

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_GAME_ABSTRACTDISPLAY_H__ */

