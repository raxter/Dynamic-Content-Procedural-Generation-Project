#ifndef __PROCGEB_ABSTRACTGAMECOMPONENT_GAME_H__
#define __PROCGEB_ABSTRACTGAMECOMPONENT_GAME_H__

#include <QObject>

#include "display.h"
#include "control_interface.h"

namespace ProcGen {

namespace AbstractGameComponent {

  
class Game : public QObject {

  Q_OBJECT

  public: /* class specific */

  Game();
  virtual ~Game();
  
  public slots: /* virtual */

  virtual void initStep();
  virtual void logicStep(const ControlInterface& controlInterface);
  virtual void renderStep(const Display& displayer);
  //TODO create cleanUpStep()

};

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */


#endif /* __PROCGEB_ABSTRACTGAME_GAME_H__ */

