#ifndef __PROCGEN_GAMECOMPONENT_TESTGAME_H__
#define __PROCGEN_GAMECOMPONENT_TESTGAME_H__

#include <QGLContext>

#include "abstract_game_components/game.h"

namespace ProcGen {

namespace GameComponent {

  
class TestGame : public AbstractGameComponent::Game {

  Q_OBJECT

  public: /* class specific */

  TestGame();
  ~TestGame();
  
  public slots: /* over-ridden */

  void initStep(void * pntr = 0);
  void logicStep(void * pntr = 0);
  void renderStep(void * pntr = 0);
  
  private:
  
  int framecount;
  
};

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */




#endif /* __GAME_TEST_H__ */

