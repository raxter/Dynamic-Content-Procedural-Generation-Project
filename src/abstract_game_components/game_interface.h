#ifndef __PROCGEB_ABSTRACTGAMECOMPONENT_GAMEINTERFACE_H__
#define __PROCGEB_ABSTRACTGAMECOMPONENT_GAMEINTERFACE_H__

#include <QObject>

#include "game.h"
#include "display.h"
#include "control_interface.h"

namespace ProcGen {

namespace AbstractGameComponent {

  
class GameInterface{


  public: /* class specific */

  GameInterface(Game& gameCore);
  virtual ~GameInterface();
  
  public: /* virtual */
  
  virtual bool initialized() = 0;

  virtual void initStep() = 0;
  virtual void cleanUpStep() = 0;
  
  virtual void logicStep() = 0;
  virtual void renderStep() = 0;
  
  
  protected: /* variables */
  
  Game& gameCore;

};

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */


#endif /* __PROCGEB_ABSTRACTGAME_GAMEINTERFACE_H__ */

