#ifndef __PROCGEN_ABSTRACTGAMECOMPONENT_CONTROLINTERFACE_H__
#define __PROCGEN_ABSTRACTGAMECOMPONENT_CONTROLINTERFACE_H__

#include <QObject>
#include <QPoint>
#include <QHash>

namespace ProcGen {

namespace AbstractGameComponent {

  
class ControlInterface {

  public: /* class specific */

  ControlInterface();
  virtual ~ControlInterface();
  
  public: /* methods */
  
  void incomingMousePosition(const QPoint& point);
  void incomingKeyEvent(int key, bool keyDown);
  void incomingMouseButtonEvent(int mouseCode, bool isMouseButtonDown);
  
  void eventStep();
  
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

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_ABSTRACTGAMECOMPONENT_CONTROLINTERFACE_H__ */

