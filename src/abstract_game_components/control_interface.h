#ifndef __PROCGEN_ABSTRACTGAMECOMPONENT_CONTROLINTERFACE_H__
#define __PROCGEN_ABSTRACTGAMECOMPONENT_CONTROLINTERFACE_H__

#include <QObject>
#include <QPoint>

namespace ProcGen {

namespace AbstractGameComponent {

  
class ControlInterface : public QObject {

  Q_OBJECT

  protected: /* class specific */

  ControlInterface();
  virtual ~ControlInterface();
  
  public: /* methods */
  
  virtual void eventStep() = 0;
  
  virtual QPoint getMousePosition() const = 0;
  
  virtual bool isKeyDown(int keyCode) const = 0;
  virtual bool isKeyTapped(int keyCode) const = 0;
  
  
};

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_ABSTRACTGAMECOMPONENT_CONTROLINTERFACE_H__ */

