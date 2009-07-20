#ifndef __PROCGEN_GAMECOMPONENT_TESTGAME_H__
#define __PROCGEN_GAMECOMPONENT_TESTGAME_H__

#include <QGLContext>
#include <QImage>
#include <QDir>

#include "abstract_game_components/game.h"

#include "Box2D.h"

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
  
  //b2Body* createBox(float32 x, float32 y, float32 width, float32 height, float32 angle = 0, float32 density = 1);
  //b2Body* createCircle(float32 x, float32 y, float32 radius, float32 angle = 0, float32 density = 1);
  //void calculateOffsetAndZoom();
  
  private:
  
  enum KeyDirection {Fore = 1, Back = 2, Left = 4, Rght = 8};
  double directionOffset [16];
  
  int framecount;
  
  double x,y,z,pitch,yaw,offx, offy, scale_zoom, display_width, display_height;
  QPoint mousePos, mouseMove;
  
  NxPhysicsSDK* gPhysicsSDK;
  NxScene* gScene;
  NxPlaneShapeDesc *planeShapeDesc;
  NxBoxShapeDesc *boxShapeDesc;
  
  /*
  b2AABB *worldAABB;
  b2World* world;
  QVector<b2Body*> bodies;
  b2Body *ball [2];
  */
  
  int launching;
  float ball_dist;
  
  float spin_force;
  
  QImage backgroundTexture;
  uint backgroundTextureId;
  
};

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */




#endif /* __GAME_TEST_H__ */

