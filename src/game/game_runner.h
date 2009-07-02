#ifndef __PROCGEN_GAME_GAMERUNNER_H__
#define __PROCGEN_GAME_GAMERUNNER_H__

#include <QThread>

#include "abstract_game.h"
#include "abstract_display.h"

namespace ProcGen {

namespace Game {

  
class GameRunner : public QThread {

  Q_OBJECT
  
  public: /* class specific */

  GameRunner(AbstractGame& gameCore, AbstractDisplay& displayer);
  ~GameRunner();
  
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

} /* end of namespace Game */

} /* end of namespace ProcGen */



#endif /* __PROCGEN_GAME_GAMERUNNER_H__ */

