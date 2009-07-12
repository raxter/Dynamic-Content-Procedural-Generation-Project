#ifndef __PROCGEN_GAMECOMPONENT_GLDISPLAYWIDGET_H__
#define __PROCGEN_GAMECOMPONENT_GLDISPLAYWIDGET_H__


#include <QGLWidget>

namespace ProcGen {

namespace GameComponent {

  
class GLDisplayWidget : public QGLWidget {

  Q_OBJECT

  public: /* class specific */

  GLDisplayWidget(QWidget* parent = 0);
  ~GLDisplayWidget();

  signals:
  
  void sendingContext();
  
  void mouseMoved(const QPoint& pos);
  void keyEvent(int key, bool keyDown);
  void mouseEvent(int button, bool buttonDown);

  public: /* over-ridden methods */
  
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

};

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */


#endif /* __PROCGEN_GUI_GLDISPLAY_H__ */

