#include "game_interface.h"

#include <QDebug>

namespace ProcGen {

namespace AbstractGameComponent {

  

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
GameInterface::GameInterface(Game& gameCore) : gameCore(gameCore)
{

}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
GameInterface::~GameInterface()
{

}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
bool GameInterface::initialized()
{
  qDebug() << "Game::initialized";
  return false;
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GameInterface::initStep()
{
  qDebug() << "Game::initStep";
}

  
/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GameInterface::cleanUpStep()
{
  qDebug() << "Game::cleanUpStep";

}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GameInterface::logicStep()
{
  qDebug() << "Game::logicStep";
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GameInterface::renderStep()
{
  qDebug() << "Game::renderStep";
}

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */

