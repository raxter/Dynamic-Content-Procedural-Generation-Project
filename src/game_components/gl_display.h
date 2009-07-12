#ifndef __PROCGEN_GAMECOMPONENT_GLDISPLAY_H__
#define __PROCGEN_GAMECOMPONENT_GLDISPLAY_H__


#include <QGLWidget>

#include "abstract_game_components/display.h"
#include "game_components/gl_display_widget.h"

namespace ProcGen {

namespace GameComponent {

  
class GLDisplay : public AbstractGameComponent::Display {

  Q_OBJECT

  public: /* class specific */

  GLDisplay(GLDisplayWidget& glDisplayWidget);
  ~GLDisplay();

  signals: /* over-ridden */
  void ready( const Display& );

  public slots: /* over-ridden */
  void requestReady();
  
  public slots:
  void sendReady();

  public: /* over-ridden methods */
  
  bool isInitialized() const;
  
  void initRenderStep();
  void cleanupRenderStep();
  
  void drawCube(double cx, double cy, double cz, double sx, double sy, double sz) const;
  void drawPolygon(const QVector<double>& points) const;
  void drawText2D(int x, int y, const QString & str, const QFont & fnt = QFont ( ), int listBase = 2000 ) const;
  //void drawMesh, etc

  private: /* variabels */
  bool initialized;
  GLDisplayWidget& glDisplayWidget;

};

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */


#endif /* __PROCGEN_GUI_GLDISPLAY_H__ */

