#include "runner.h"

#include <QDebug>

namespace ProcGen {

namespace AbstractGameComponent {


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
Runner::Runner(Game& gameCore, Display& displayer, const ControlInterface& controlInterface)
{
  
  connect(this, SIGNAL(doInitStep()), &gameCore, SLOT(initStep()));
  
  connect(this, SIGNAL(doLogicStep()), &controlInterface, SLOT(requestReady()));
  connect(&controlInterface, SIGNAL(ready( const ControlInterface& )), &gameCore, SLOT(logicStep(const ControlInterface& )));
  
  //connect(this, SIGNAL(doRenderStep()), &gameCore, SLOT(renderStep()));
  
  connect(this, SIGNAL(doRenderStep()), &displayer, SLOT(requestReady()));
  connect(&displayer, SIGNAL(ready( const Display& )), &gameCore, SLOT(renderStep( const Display& )));
  
  connect(this, SIGNAL(doEventStep()), &controlInterface, SLOT(eventStep()));
  
  //TODO, implement this
  //connect(&displayer, SIGNAL(displayInitialized()), this, SLOT(displayerInitialized()));
  
  
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
Runner::~Runner()
{
  
}


/****************************************************************************
**
** Author: Richard Baxter
**
** FIXME AbstractGame& abstractGame maybe should go in constructor
**
****************************************************************************/
void Runner::runGame()
{
  /* TODO do some stuff to see if game can be run and whatnot, do initialisation of game, make sure thread already isn't running! etc */
  /* even though the displayInitialized slot is to be called, double check if it has not been initilized already*/
  QThread::start(); // calls run()
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Runner::start() {
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Runner::displayerInitialized() {
  running = true;
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Runner::run()
{
  /* TODO, proper game loop goes here */
  
  running = false;
  emit doInitStep();
  
  while (running) {
    ;//yieldCurrentThread ();
  }
  
  for (int i = 0 ; i < 10000 ; i++) {
    qDebug() << "step " << i;
    emit doEventStep();
    emit doLogicStep();
    emit doRenderStep();
    usleep(50000);
  }
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Runner::normalQuit()
{
  /* TODO set the running variable to false or whatever :/ */
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Runner::forceQuit()
{
  quit(); /* force closes the thread */
}

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */

