#include "test_game.h"

//#include "abstract_game_components/game.h"
//#include "abstract_game_components/display.h"

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
  qDebug() << "TestGame::initStep" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
  
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
  sceneDesc.simType = NX_SIMULATION_SW; // Software mode
  //sceneDesc.simType = NX_SIMULATION_HW; // Hardware mode
  
  gScene = gPhysicsSDK->createScene(sceneDesc);
  qDebug() << "gScene = " << gScene << ", sim type = " << gScene->getSimType () << ", writable? = " <<  gScene->isWritable ();
  
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
  
  
  for (int i = 0 ; i < 300 ; i++) {
    // create box
    
	  //Define new physics object
	  NxBodyDesc bodyDesc;
	  bodyDesc.angularDamping	= 0.5f;
	  bodyDesc.linearVelocity = NxVec3(0.0f, 0.0f, 0.0f);

	  //Decribe shape
    NxBoxShapeDesc boxShapeDesc;
    boxShapeDesc.dimensions = NxVec3(1,1,1);
    
    NxActorDesc boxActorDesc;
    boxActorDesc.shapes.push_back(&boxShapeDesc);
	  boxActorDesc.body = &bodyDesc;
	  boxActorDesc.density	= 10.0f;
	  boxActorDesc.globalPose.t = NxVec3(0, 2+i*2, 0);
    box = gScene->createActor(boxActorDesc);
  }
  
  
	//NxReal timeStep = 1.0f / 60.0f;
  //gScene->simulate (timeStep);
 
  //gScene->setTiming(1.0/60.0);
	
  backgroundTexture = QImage("../content/background.png");
  backgroundTextureId = displayer.bindTexture(backgroundTexture);
  glBindTexture(GL_TEXTURE_2D, -1);
  
  
    // Update the time step
    NxReal deltaTime = 1.0f/60.0f;

    // Start collision and dynamics for delta time since the last frame
    gScene->simulate(deltaTime);
    gScene->flushStream(); 
  
  qDebug() << "END TestGame::initStep" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
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
void TestGame::logicStep(const AbstractGameComponent::ControlInterface& controlInterface)
{
   while (!gScene->fetchResults(NX_RIGID_BODY_FINISHED, false)); 

  //qDebug() << "TestGame::logicStep" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
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
    qDebug() << directionCode << ":" << directionOffset[directionCode];
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

  box->addForce(NxVec3(0,1,0));
  //gScene->flushStream();
  //gScene->fetchResults(NX_RIGID_BODY_FINISHED, true); 
  
	//NxReal timeStep = 1.0f / 60.0f;
  //gScene->simulate (timeStep);
  
    // Update the time step
    NxReal deltaTime = 1.0f/60.0f;
  
    //qDebug() << "gScene = " << gScene << ", sim type = " << gScene->getSimType () << ", writable? = " <<  gScene->isWritable ();
    // Start collision and dynamics for delta time since the last frame
    gScene->simulate(deltaTime);
    gScene->flushStream(); 

  //qDebug() << "ENDTestGame::logicStep" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::renderStep(const AbstractGameComponent::Display& displayer)
{


  //qDebug() << "TestGame::renderStep" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
  //qDebug() << "currentContext valid: " << QGLContext::currentContext ()->isValid();

  //TODO there should be NO gl in this class
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); 
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  
  //glRotated(90.0/M_PI*180,0,1,0);
  glRotated(pitch/M_PI*180, 1,0,0);
  glRotated(yaw/M_PI*180, 0,1,0);
  glTranslated(x,y,z);
  
  
  
  glBindTexture(GL_TEXTURE_2D, backgroundTextureId);
  
  glColor3d(1.0,1.0,1.0);
  
  /*float size = 1000;
  glBegin(GL_QUADS);
  for (int i = -1 ; i <= 0 ; i++) {
    for (int j = -0 ; j <= 0 ; j++) {
      glTexCoord2f(1,0); glVertex3f(size*(i),   size*(j),   -10);
      glTexCoord2f(1,1); glVertex3f(size*(i),   size*(j+1), -10);
      glTexCoord2f(0,1); glVertex3f(size*(i+1), size*(j+1), -10);
      glTexCoord2f(0,0); glVertex3f(size*(i+1), size*(j),   -10);
    }
  }
  glEnd();*/
  glBindTexture(GL_TEXTURE_2D, -1);
  
  glColor3d(1.0,1.0,1.0);
  
  //qDebug() << "actors";
  int nbActors = gScene->getNbActors();
  NxActor** actors = gScene->getActors();
  while (nbActors--)
  {
    NxActor* actor = *actors++;
    //qDebug() << actor;
    NxVec3 pos = actor->getGlobalPosition ();
    //qDebug() << pos.x << ":" << pos.y << ":" << pos.z << ", mass = " << actor->getMass ();
    displayer.drawActor(actor);
    //DrawActor(actor);
  } 
 
  //qDebug() << "BOX";
  //NxVec3 pos = box->getGlobalPosition ();
  //qDebug() << pos.x << ":" << pos.y << ":" << pos.z;
	
  glColor3d(1.0,1.0,1.0);
  //displayer.drawText2D(10,30, QString("Ang Vel = ") + QString::number(ball[0]->GetAngularVelocity()), QFont());
  displayer.drawText2D(10,50, QString("P,Y ") + QString::number(pitch) +", "+QString::number(yaw), QFont());
  displayer.drawText2D(10,70, QString("Pos ") + QString::number(x) +", "+QString::number(y) +", "+QString::number(z), QFont());
  
  
  //qDebug() << "END TestGame::renderStep" << "currentContext: " << QGLContext::currentContext () << " Thread: " << QThread::currentThread ();
}

} /* end of namespace GameComponent */

} /* end of namespace ProcGen */

