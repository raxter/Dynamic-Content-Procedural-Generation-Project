#ifndef __PROCGEN_ABSTRACTGAMECOMPONENT_RUNNER_H__
#define __PROCGEN_ABSTRACTGAMECOMPONENT_RUNNER_H__

#include <QThread>

#include "game.h"
#include "display.h"

namespace ProcGen {

namespace AbstractGameComponent {

class Runner : public QThread {

  Q_OBJECT
  
  public: /* class specific */

  Runner(Game& gameCore, Display& displayer);
  ~Runner();
  
  public slots:

  void runGame();
  void displayerInitialized();
  
  signals:
  
  void doInitStep();
  void doLogicStep();
  void doRenderStep();
  
  void requestDisplayInitializationSignal();
  
  private: /* methods */
  void start();
  void run();
  
  void normalQuit();
  void forceQuit();
  
  private: /* variables */
  bool running;
  
};

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_GAME_GAMERUNNER_H__ */

