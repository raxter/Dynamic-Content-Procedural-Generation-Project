#ifndef __PROCGEB_ABSTRACTGAMECOMPONENT_GAME_H__
#define __PROCGEB_ABSTRACTGAMECOMPONENT_GAME_H__

#include <QObject>

#include "display.h"
#include "control_interface.h"

namespace ProcGen {

namespace AbstractGameComponent {

  
class Game{


  public: /* class specific */

  Game();
  virtual ~Game();
  
  public: /* virtual */

  virtual void resizeStep(int width, int height) = 0;
  virtual void initStep(const Display& displayer) = 0;
  virtual void logicStep(const ControlInterface& controlInterface) = 0;
  virtual void renderStep(const Display& displayer) = 0;
  virtual void cleanUpStep() = 0;

};

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */


#endif /* __PROCGEB_ABSTRACTGAME_GAME_H__ */

