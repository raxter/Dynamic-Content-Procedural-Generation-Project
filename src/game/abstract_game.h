#ifndef __PROCGEB_GAME_ABSTRACTGAME_H__
#define __PROCGEB_GAME_ABSTRACTGAME_H__


namespace ProcGen {

namespace Game {

  
class AbstractGame {

  public: /* class specific */

  AbstractGame();
  virtual ~AbstractGame();
  
  public: /* virtual methods */

  virtual void initStep(void * pntr = 0);
  virtual void logicStep(void * pntr = 0);
  virtual void renderStep(void * pntr = 0);

};

} /* end of namespace Game */

} /* end of namespace ProcGen */


#endif /* __PROCGEB_ABSTRACTGAME_GAME_H__ */

