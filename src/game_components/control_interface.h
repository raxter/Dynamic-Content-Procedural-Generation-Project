#ifndef __PROCGEN_GAMECOMPONENT_CONTROLINTERFACE_H__
#define __PROCGEN_GAMECOMPONENT_CONTROLINTERFACE_H__

#include <QObject>
#include <QPoint>
#include <QWidget>
#include <QList>
#include <QHash>

#include "abstract_game_components/control_interface.h"

namespace ProcGen {

namespace GameComponent {

  
class ControlInterface : public AbstractGameComponent::ControlInterface {

  Q_OBJECT

  public: /* class specific */

  ControlInterface(const QWidget * eventSender);
  virtual ~ControlInterface();
  
  public slots:
  
  void eventStep();
  
  private slots:
  
  void incomingMousePosition(const QPoint& point);
  void incomingKeyEvent(int key, bool keyDown);
  void incomingMouseButtonEvent(int mouseCode, bool isMouseButtonDown);
  
  public: /* methods */
  
  QPoint getMousePosition() const;
  QPoint getMouseMovement() const;
  
  bool isKeyDown(int keyCode) const;
  bool isKeyTapped(int keyCode) const;
  
  bool isMouseDown(int mouseButtonCode) const;
  bool isMouseTapped(int mouseButtonCode) const;
  
  
  private: /* variables */
  
  QPoint mousePosition;
  QPoint mouseMovement;
  QPoint currentMousePosition;
  QPoint oldMousePosition;
  
  // TODO make into hashes
  QHash<int, bool> keyDown;
  QHash<int, bool> keyTapped;
  QHash<int, bool> keyJustDown;
  
  QList<int> keysUp, keysDown, forceTapUp;
  
  // TODO make into hashes
  QHash<int, bool> mouseDown;
  QHash<int, bool> mouseTapped;
  QHash<int, bool> mouseJustDown;
  
  QList<int> mouseButtonsUp, mouseButtonsDown, forceMouseTapUp;
  
};

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_ABSTRACTGAMECOMPONENT_CONTROLINTERFACE_H__ */

