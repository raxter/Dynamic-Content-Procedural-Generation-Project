#ifndef __PROCGEN_GAMECOMPONENT_TESTGAME_H__
#define __PROCGEN_GAMECOMPONENT_TESTGAME_H__

#include <QImage>
#include <QDir>

#include "abstract_game_components/game.h"


#include "NxPhysics.h"


namespace ProcGen {

namespace GameComponent {

  
class TestGame : public AbstractGameComponent::Game {


  public: /* class specific */

  TestGame();
  virtual ~TestGame();
  
  public slots: /* over-ridden */

  virtual void resizeStep(int width, int height);
  virtual void initStep(const AbstractGameComponent::Display& displayer);
  virtual void logicStep(const AbstractGameComponent::ControlInterface& controlInterface);
  virtual void renderStep(const AbstractGameComponent::Display& displayer);
  virtual void cleanUpStep();
  
  private: /* methods */
  
  NxActor* createBox(NxVec3 pos, NxVec3 dim);
  
  private:
  
  enum KeyDirection {Fore = 1, Back = 2, Left = 4, Rght = 8};
  double directionOffset [16];
  
  int framecount;
  
  double x,y,z,pitch,yaw, display_width, display_height;
  QPoint mousePos, mouseMove;
  
  NxPhysicsSDK* gPhysicsSDK;
  NxScene* gScene;
  
  
  
  QImage backgroundTexture;
  uint backgroundTextureId;
  
};

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */




#endif /* __GAME_TEST_H__ */

