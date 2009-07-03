#include "test_game.h"

//#include "abstract_game_components/game.h"
//#include "abstract_game_components/display.h"

#include <QDebug>

namespace ProcGen {

namespace GameComponent {

  

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
TestGame::TestGame() : AbstractGameComponent::Game(), framecount(0)
{

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
void TestGame::initStep(void * pntr)
{
  qDebug() << "TestGame::initStep";
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::logicStep(void * pntr)
{
  qDebug() << "TestGame::logicStep";
  framecount++;
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void TestGame::renderStep(void * pntr)
{

  qDebug() << "TestGame::renderStep";
  

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  //glTranslated(300,300,0);
  glRotated(45, 1,0,0);
  glRotated(45, 0,1,0);

  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); 
  
  {
    /*glColor3f(0.0,0.0,1.0);
    QVector<double> points;
    points.append(  0.1); points.append(  0.1); points.append(0.5);
    
    points.append( 100); points.append(  0.1); points.append(0.5);
    
    points.append( 100); points.append( 100); points.append(0.5);
    
    points.append(  0.1); points.append( 100); points.append(0.5);
     
    displayer->drawPolygon(points);*/
    displayer->drawCube(0,0,0,0,0,0);
    
  }
}


} /* end of namespace GameComponent */

} /* end of namespace ProcGen */

