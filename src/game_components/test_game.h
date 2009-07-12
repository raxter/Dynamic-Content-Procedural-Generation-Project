#ifndef __PROCGEN_GAMECOMPONENT_TESTGAME_H__
#define __PROCGEN_GAMECOMPONENT_TESTGAME_H__

#include <QGLContext>

#include "pal/pal/palFactory.h"

#include "abstract_game_components/game.h"

namespace ProcGen {

namespace GameComponent {

  
class TestGame : public AbstractGameComponent::Game {

  Q_OBJECT

  public: /* class specific */

  TestGame();
  ~TestGame();
  
  public slots: /* over-ridden */

  void initStep();
  void logicStep(const AbstractGameComponent::ControlInterface& controlInterface);
  void renderStep(const AbstractGameComponent::Display& displayer);
  
  private:
  
  enum KeyDirection {Fore = 1, Back = 2, Left = 4, Rght = 8};
  double directionOffset [16];
  
  int framecount;
  
  double x, y, z, pitch, yaw;
  QPoint mousePos, mouseMove;
  
  
	palPhysics *pp;
	
	palTerrainPlane *pt;
	palBox *pb;
  
};

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */




#endif /* __GAME_TEST_H__ */

