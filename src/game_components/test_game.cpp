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

