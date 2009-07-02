#ifndef __PROCGEN_GUI_GLDISPLAYWIDGET_H__
#define __PROCGEN_GUI_GLDISPLAYWIDGET_H__


#include <QGLWidget>

namespace ProcGen {

namespace GUI {

  
class GLDisplayWidget : public QGLWidget {

  Q_OBJECT

  public: /* class specific */

  GLDisplayWidget(QWidget* parent = 0);
  ~GLDisplayWidget();

  signals:
  
  void sendingContext();

  public: /* over-ridden methods */
  
  void initializeGL();
  void paintGL();
  void resizeGL ( int width, int height );

};

} /* end of namespace GUI */

} /* end of namespace ProcGen */


#endif /* __PROCGEN_GUI_GLDISPLAY_H__ */

