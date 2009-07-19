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
void Game::resizeStep(int width, int height)
{
  qDebug() << "Game::resizeStep";
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Game::initStep(const Display& displayer)
{
  qDebug() << "Game::initStep";
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
  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Game::cleanUpStep()
{
  qDebug() << "Game::cleanUpStep";

}

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */

