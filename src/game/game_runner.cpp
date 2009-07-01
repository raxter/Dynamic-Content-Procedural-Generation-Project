#include <game_runner.h>

namespace ProcGen {

namespace Game {


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
GameRunner::GameRunner();
{
  
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
GameRunner::~GameRunner();
{
  
}


/****************************************************************************
**
** Author: Richard Baxter
**
** FIXME AbstractGame& abstractGame maybe should go in constructor
**
****************************************************************************/
void GameRunner::runAbstractGame(GLDisplay& displayer)
{
  /* TODO do some stuff to see if game can be run and whatnot, do initialisation of game, make sure thread already isn't running! etc */
  start(); // calls run()
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GameRunner::run()
{
  /* TODO, game loop goes here */
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GameRunner::normalQuit()
{
  /* TODO set the running variable to false or whatever :/ */
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void GameRunner::forceQuit()
{
  quit(); /* force closes the thread */
}

} /* end of namespace Game */

} /* end of namespace ProcGen */

