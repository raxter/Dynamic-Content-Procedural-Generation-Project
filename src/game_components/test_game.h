#ifndef __GAME_TEST_H__
#define __GAME_TEST_H__

#include <QGLContext>

#include "abstract_game_components/game.h"

namespace ProcGen {

namespace Game {

  
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

} /* end of namespace Game */

} /* end of namespace ProcGen */




#endif /* __GAME_TEST_H__ */

