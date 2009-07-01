#ifndef __PROCGEN_GAME_GAMERUNNER_H__
#define __PROCGEN_GAME_GAMERUNNER_H__

#include <QThread>

namespace ProcGen {

namespace Game {

  
class GameRunner : private QThread {

  public: /* class specific */

  GameRunner();
  ~GameRunner();
  
  public: /* methods */

  void runAbstractGame(AbstractGame& abstractGame);
  void run();
  
  void normalQuit();
  void forceQuit();
  
  
};

} /* end of namespace Game */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_GAME_GAMERUNNER_H__ */

