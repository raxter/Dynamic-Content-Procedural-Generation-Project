#include "test_game.h"

//#include "abstract_game_components/game.h"
//#include "abstract_game_components/display.h"

#include <QDebug>


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
  //TODO move to (non-existant) cleanUpStep()	
  PF->Cleanup(); //we are done with the physics. clean up.
  delete pp;
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::initStep()
{
  qDebug() << "TestGame::initStep";
  
  x = 0; y = 0; z = 0;
  
  PF->LoadPALfromDLL(); 
  
  PF->SelectEngine("Bullet"); //"Bullet" is the name of the engine you wish to use. eg:"Bullet"
	pp = PF->CreatePhysics(); //create the main physics class
  if (pp == NULL) {
		printf("Failed to create the physics engine. Check to see if you spelt the engine name correctly, and that the engine DLL is in the right location");
	}
	else {
    pp->Init(0,-9.8f,0); //initialize it, set the main gravity vector
	  pt= PF->CreateTerrainPlane(); //create the ground
	  pt->Init(0,0,0,50.0f); //initialize it, set its location to 0,0,0 and minimum size to 50
	  pb = PF->CreateBox(); //create a box
	  pb->Init(0,5,0, 1,1,1, 1); //initialize it, set its location to 0,5,0 (five units up in the air), set dimensions to 1x1x1 and its mass to 1
  }

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
    qDebug() << directionCode << ":" << directionOffset[directionCode];
    double directionToMove = yaw + directionOffset[directionCode];
    x -= 10*sin(directionToMove);
    y += 10*cos(directionToMove);
  }
  
  if  (controlInterface.isKeyDown(Qt::Key_Space))
    z += 10;
  if  (controlInterface.isKeyDown(Qt::Key_Shift))
    z -= 10;
    
  if (controlInterface.isMouseDown(Qt::LeftButton)) {
    pitch += 0.03*mouseMove.y();
    yaw += 0.03*mouseMove.x();
  }
  
  
  pp->Update(0.02f); //update the physics engine. advance the simulation time by 0.02

}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::renderStep(const AbstractGameComponent::Display& displayer)
{

  //qDebug() << "TestGame::renderStep";
  //qDebug() << displayer;
  palVector3 pos;
  pb->GetPosition(pos); //get the location of the box

  printf("Current box position is %6.5f at time %4.2f\n",pos.y,pp->GetTime());


  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  glRotated(90.0/M_PI*180,1,0,0);
  glRotated(pitch/M_PI*180, 1,0,0);
  glRotated(-yaw/M_PI*180, 0,0,1);
  glTranslated(x,y,z);

  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); 
  
  {
    /*glColor3f(0.0,0.0,1.0);
    QVector<double> points;
    points.append(  0.1); points.append(  0.1); points.append(0.5);
    
    points.append( 100); points.append(  0.1); points.append(0.5);
    
    points.append( 100); points.append( 100); points.append(0.5);
    
    points.append(  0.1); points.append( 100); points.append(0.5);
     
    displayer->drawPolygon(points);*/
    
  }
  //glTranslate(100,0,0);
  glScaled(10,10,10);
  displayer.drawCube(0,0,0,0,0,0);
  
  
  glColor3d(1.0,1.0,1.0);
  displayer.drawText2D(10,10, "Pitch " + QString::number(pitch), QFont());
  displayer.drawText2D(10,30, "Yaw " + QString::number(yaw), QFont());
}


} /* end of namespace GameComponent */

} /* end of namespace ProcGen */

