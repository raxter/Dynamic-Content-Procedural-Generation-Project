#ifndef __PROCGEN_GAMECOMPONENT_GLDISPLAYWIDGET_H__
#define __PROCGEN_GAMECOMPONENT_GLDISPLAYWIDGET_H__


#include <QGLWidget>
#include <QMouseEvent>
#include <QKeyEvent>

#include "abstract_game_components/game_interface.h"
#include "abstract_game_components/control_interface.h"

namespace ProcGen {

namespace GameComponent {

  
class GLDisplayWidget : public QGLWidget, public AbstractGameComponent::GameInterface {

  Q_OBJECT

  public: /* class specific */

  GLDisplayWidget(AbstractGameComponent::Game& gameCore, AbstractGameComponent::Display& displayer, AbstractGameComponent::ControlInterface& controlInterface, QWidget* parent = 0);
  ~GLDisplayWidget();
  
  signals:

  void mouseMoved(const QPoint& pos);
  void keyEvent(int key, bool keyDown);
  void mouseEvent(int button, bool buttonDown);

  public: /* over-ridden methods */
  
  /* from GameInterface */
  void initStep();
  void logicStep();
  void renderStep();
  void cleanUpStep();
  bool initialized();
 
  /* from QGLWidget */
  void initializeGL();
  void paintGL();
  void resizeGL ( int width, int height );
  
  /* from QWidget */
  void mouseMoveEvent ( QMouseEvent * event );
  void wheelEvent ( QWheelEvent * event );
  void keyPressEvent ( QKeyEvent * event );
  void keyReleaseEvent ( QKeyEvent * event );
  void mousePressEvent ( QMouseEvent * event );
  void mouseReleaseEvent ( QMouseEvent * event );
  
  private:
  
  bool performResize, isInitialized;
  int resize_width, resize_height;
  
  AbstractGameComponent::Display& displayer;
  AbstractGameComponent::ControlInterface& controlInterface;

};

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */


#endif /* __PROCGEN_GUI_GLDISPLAY_H__ */

