#ifndef __PROCGEN_GUI_GLDISPLAY_H__
#define __PROCGEN_GUI_GLDISPLAY_H__


#include <QGLWidget>

#include "game/abstract_display.h"
#include "gl_display_widget.h"

namespace ProcGen {

namespace GUI {

  
class GLDisplay : public Game::AbstractDisplay {

  Q_OBJECT

  public: /* class specific */

  GLDisplay(GLDisplayWidget& glDisplayWidget);
  ~GLDisplay();

  signals: /* over-ridden */
  void ready();

  public slots: /* over-ridden */
  void requestReady();

  public: /* over-ridden methods */
  
  bool isInitialized() const;
  
  void initRenderStep();
  void cleanupRenderStep();
  
  void drawCube(double cx, double cy, double cz, double sx, double sy, double sz);
  void drawPolygon(const QVector<double>& points);
  //void drawMesh, etc

  private: /* variabels */
  bool initialized;
  GLDisplayWidget& glDisplayWidget;

};

} /* end of namespace GUI */

} /* end of namespace ProcGen */


#endif /* __PROCGEN_GUI_GLDISPLAY_H__ */

