#ifndef __PROCGEN_GAMECOMPONENT_CONTROLINTERFACE_H__
#define __PROCGEN_GAMECOMPONENT_CONTROLINTERFACE_H__

#include <QObject>
#include <QPoint>
#include <QWidget>

#include "abstract_game_components/control_interface.h"

namespace ProcGen {

namespace GameComponent {

  
class ControlInterface : public AbstractGameComponent::ControlInterface {

  Q_OBJECT

  public: /* class specific */

  ControlInterface(const QWidget * eventSender);
  virtual ~ControlInterface();
  
  public slots:
  
  void incomingMousePosition(const QPoint& point);
  
  public slots:
  
  void eventStep();
  
  public: /* methods */
  
  QPoint getMousePosition() const;
  
  bool isKeyDown(int keyCode) const;
  bool isKeyTapped(int keyCode) const;
  
  
  private: /* variables */
  
  QPoint mousePosition;
  QPoint newMousePosition;
  
};

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_ABSTRACTGAMECOMPONENT_CONTROLINTERFACE_H__ */

