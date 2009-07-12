#include "game.h"

#include <QDebug>

namespace ProcGen {

namespace AbstractGameComponent {

  

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
Game::Game()
{

}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
Game::~Game()
{

}
  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Game::initStep()
{

}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Game::logicStep(const ControlInterface& controlInterface)
{
  qDebug() << "Game::logicStep";
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Game::renderStep(const Display& displayer )
{
  qDebug() << "Game::renderStep";
}

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */

