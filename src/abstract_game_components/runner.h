#ifndef __PROCGEN_ABSTRACTGAMECOMPONENT_RUNNER_H__
#define __PROCGEN_ABSTRACTGAMECOMPONENT_RUNNER_H__

#include <QThread>
#include <QGLContext>       
#include <sys/time.h> 

#include "game_interface.h"

namespace ProcGen {

namespace AbstractGameComponent {

class Runner : public QThread {

  Q_OBJECT
  
  public: /* class specific */

  Runner(GameInterface& gameInterface);
  ~Runner();

  public: /* methods */
  
  void runGame();
  
  private: /* methods */
  void start();
  void run();
  
  void normalQuit();
  void forceQuit();
  
  private: /* variables */
  
  
  bool running;
  
  GameInterface& gameInterface;
  
  
  struct timeval tv;
  unsigned long long int getTimeOfDay();

  
};

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_GAME_GAMERUNNER_H__ */

