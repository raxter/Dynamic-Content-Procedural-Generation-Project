#ifndef __PROCGEN_GUI_GLDISPLAY_H__
#define __PROCGEN_GUI_GLDISPLAY_H__


#include <QGLWidget>

namespace ProcGen {

namespace GUI {

  
class GLDisplay : public QGLWidget {

  //Q_OBJECT

  public: /* class specific */

  GLDisplay(QWidget* parent = 0);
  ~GLDisplay();

  public: /* over-rided methods */
  
  void initializeGL();
  //void paintGL(); /*NOTE don't use paintGL because it is called by qt automatically, and we don't want that...*/
  void updateGL();
  void resizeGL();


};

} /* end of namespace GUI */

} /* end of namespace ProcGen */


#endif /* __PROCGEN_GUI_GLDISPLAY_H__ */

