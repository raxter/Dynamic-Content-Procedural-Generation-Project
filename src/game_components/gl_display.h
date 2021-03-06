#ifndef __PROCGEN_GAMECOMPONENT_GLDISPLAY_H__
#define __PROCGEN_GAMECOMPONENT_GLDISPLAY_H__


#include <QGLWidget>

#include "abstract_game_components/display.h"
#include "game_components/gl_display_widget.h"

namespace ProcGen {

namespace GameComponent {

  
class GLDisplay : public AbstractGameComponent::Display {


  public: /* class specific */

  GLDisplay();
  ~GLDisplay();


  public: /* over-ridden methods */
  
  void setGLDisplayWidget(GLDisplayWidget* glDisplayWidget);
  
  void initialize();
  
  void resize(int width, int height);
  
  void initRenderStep();
  void cleanupRenderStep();
  
  void setUpCamera(double x, double y, double z, double pitch, double yaw) const;
  
  //unsigned int bindTexture(const QImage& image) const;
  
  unsigned int loadTexture(const QImage& image) const;
  void bindTexture(unsigned int id) const;
  
  //void drawBody(b2Body* body) const;
  void drawActor(NxActor* actor) const;
  
  bool closeTo (NxReal r, NxReal val) const;
  void drawPlane(NxPlaneShape* plane) const;
  void drawBox(NxBoxShape* boxShape) const;
  
  
  void setColour(double red, double green, double blue, double alpha) const;
  
  void drawText2D(int x, int y, const QString & str, const QFont & fnt = QFont ( ), int listBase = 2000 ) const;
  //void drawMesh, etc

  private: /* variabels */
  bool initialized;
  GLDisplayWidget* glDisplayWidget;

};

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */


#endif /* __PROCGEN_GUI_GLDISPLAY_H__ */

