#include "test_game.h"

#include <QDebug>
#include <QThread>


#include <cmath>

namespace ProcGen {

namespace GameComponent {

  

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
TestGame::TestGame() : AbstractGameComponent::Game(), framecount(0)
{
  directionOffset [0] = 0;
  directionOffset [Fore|Back] = 0;
  directionOffset [Left|Rght] = 0;
  directionOffset [Fore|Back|Left|Rght] = 0;
  
  directionOffset [Fore] = 0;
  directionOffset [Back] = 2*M_PI*2/4;
  directionOffset [Left] = 2*M_PI*3/4;
  directionOffset [Rght] = 2*M_PI*1/4;
  
  directionOffset [Fore|Left] = 2*M_PI*7/8;
  directionOffset [Fore|Rght] = 2*M_PI*1/8;
  directionOffset [Back|Left] = 2*M_PI*5/8;
  directionOffset [Back|Rght] = 2*M_PI*3/8;
  
  
  directionOffset [Fore|Left|Rght] = directionOffset [Fore];
  directionOffset [Back|Left|Rght] = directionOffset [Back];
  directionOffset [Left|Fore|Back] = directionOffset [Left];
  directionOffset [Rght|Fore|Back] = directionOffset [Rght];
  
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
TestGame::~TestGame()
{
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::resizeStep(int width, int height) {
  display_width = width;
  
  display_height = height;
  
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::initStep(const AbstractGameComponent::Display& displayer)
{
  qDebug() << "TestGame::initStep";
  
  x = 0; y = -10; z = -50, pitch = 0, yaw = 0;
  
  
  gPhysicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION);
  qDebug() << "gPhysicsSDK = " << gPhysicsSDK << ", HW vers = " << gPhysicsSDK->getHWVersion () ;
  
  // Set scale dependent parameters
  NxReal scale = 1.0f;   // scale is meters per PhysX units

  gPhysicsSDK->setParameter(NX_SKIN_WIDTH, 0.05*(1/scale));
  gPhysicsSDK->setParameter(NX_DEFAULT_SLEEP_LIN_VEL_SQUARED, 0.15*0.15*(1/scale)*(1/scale));
  gPhysicsSDK->setParameter(NX_BOUNCE_THRESHOLD, -2*(1/ scale));
  gPhysicsSDK->setParameter(NX_VISUALIZATION_SCALE, 0.5*(1/ scale)); 
  
  NxSceneDesc sceneDesc;
  sceneDesc.gravity.set(0.0f, -9.8f, 0.0f);
  //sceneDesc.simType = NX_SIMULATION_SW; // Software mode
  sceneDesc.simType = NX_SIMULATION_HW; // Hardware mode
  
  gScene = gPhysicsSDK->createScene(sceneDesc);
  qDebug() << "gScene = " << gScene << ", sim type = " << gScene->getSimType () << ", writable? = " <<  gScene->isWritable ()<< ", SimType = " << gScene->getSimType () ;
  
  //Set default material
	NxMaterial* defaultMaterial = gScene->getMaterialFromIndex(0);
	defaultMaterial->setRestitution(0.0f);
	defaultMaterial->setStaticFriction(0.5f);
	defaultMaterial->setDynamicFriction(0.5f);

  
	//Create ground plane
  NxActorDesc planeActorDesc;
  NxPlaneShapeDesc planeShapeDesc;
  planeActorDesc.shapes.push_back(&planeShapeDesc);
  gScene->createActor(planeActorDesc);
  
  
  // create boxs
  for (int k = 0 ; k < 5 ; k++) {
    for (int j = 0 ; j < 5 ; j++) {
      for (int i = 0 ; i < 40 ; i++) {
        createBox(NxVec3(j*2, 2+i*2, k*2), NxVec3(1.0f, 1.0f, 1.0f));
      }
    }
  }
  
  
  
  backgroundTexture = QImage("../content/background.png");
  backgroundTextureId = displayer.loadTexture(backgroundTexture);
  
  
  qDebug() << "END TestGame::initStep";
}

void TestGame::cleanUpStep() {
  gPhysicsSDK->releaseScene(*gScene);
  gPhysicsSDK->release();
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/

NxActor* TestGame::createBox(NxVec3 pos, NxVec3 dim)
{
  
  //Define new physics object
  NxBodyDesc bodyDesc;
  bodyDesc.angularDamping	= 0.5f;
  bodyDesc.linearVelocity = NxVec3(0.0f, 0.0f, 0.0f);

  //Decribe shape
  NxBoxShapeDesc boxShapeDesc;
  boxShapeDesc.dimensions = dim;
  
  NxActorDesc boxActorDesc;
  boxActorDesc.shapes.push_back(&boxShapeDesc);
  boxActorDesc.body = &bodyDesc;
  boxActorDesc.density	= 10.0f;
  boxActorDesc.globalPose.t = pos;
  
  NxActor* box = gScene->createActor(boxActorDesc);
  
  return box;
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::logicStep(const AbstractGameComponent::ControlInterface& controlInterface)
{

  //qDebug() << "TestGame::logicStep";
  framecount++;
  
  mousePos = controlInterface.getMousePosition();
  mouseMove = controlInterface.getMouseMovement();
  

  int directionCode = 0;
  if (controlInterface.isKeyDown(Qt::Key_W))
    directionCode |= Fore;
  if (controlInterface.isKeyDown(Qt::Key_S))
    directionCode |= Back;
  if (controlInterface.isKeyDown(Qt::Key_A))
    directionCode |= Left;
  if (controlInterface.isKeyDown(Qt::Key_D))
    directionCode |= Rght;
    
  if (directionCode){
    double directionToMove = yaw + directionOffset[directionCode];
    x -= 0.1*sin(directionToMove);
    z += 0.1*cos(directionToMove);
  }
  
  if  (controlInterface.isKeyDown(Qt::Key_Space))
    y -= 0.1;
  if  (controlInterface.isKeyDown(Qt::Key_Shift))
    y += 0.1;
    
  if (controlInterface.isMouseDown(Qt::LeftButton)) {
    pitch += 0.01*mouseMove.y();
    yaw += 0.01*mouseMove.x();
  }
  
  // Update the time step
  NxReal deltaTime = 1.0f/60.0f;

  // Start collision and dynamics for delta time since the last frame
  gScene->simulate(deltaTime);
  gScene->flushStream(); 
  gScene->fetchResults(NX_RIGID_BODY_FINISHED, true); 

  //qDebug() << "ENDTestGame::logicStep";
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::renderStep(const AbstractGameComponent::Display& displayer)
{
  //qDebug() << "TestGame::renderStep";
  displayer.bindTexture();
  displayer.setUpCamera(x,y,z,pitch,yaw);
  
  
  int nbActors = gScene->getNbActors();
  NxActor** actors = gScene->getActors();
  while (nbActors--)
  {
    NxActor* actor = *actors++;
    displayer.drawActor(actor);
  }
	
  displayer.setColour(1.0,1.0,1.0);
  displayer.drawText2D(10,10, QString("P,Y ") + QString::number(pitch) +", "+QString::number(yaw), QFont());
  displayer.drawText2D(10,30, QString("Pos ") + QString::number(x) +", "+QString::number(y) +", "+QString::number(z), QFont());
  
  
  //qDebug() << "END TestGame::renderStep";
}

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */

