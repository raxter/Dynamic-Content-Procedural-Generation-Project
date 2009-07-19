#include "runner.h"

#include <QDebug>
#include <QGLFormat>

namespace ProcGen {

namespace AbstractGameComponent {


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
Runner::Runner(GameInterface& gameInterface) : gameInterface(gameInterface)
{
  
  //connect(this, SIGNAL(requestGLContext()), GLContextGiver, SLOT(incomingGLContextRequest()), Qt::DirectConnection);
  //connect(GLContextGiver, SIGNAL(outgoingGLContext()), this, SLOT(incomingGLContext()), Qt::DirectConnection);
  
  //connect(this, SIGNAL(doInitStep()), &gameCore, SLOT(initStep()));
  
  //connect(this, SIGNAL(doLogicStep()), &controlInterface, SLOT(requestReady()));
  //connect(&controlInterface, SIGNAL(ready( const ControlInterface& )), &gameCore, SLOT(logicStep(const ControlInterface& )));
  
  //connect(this, SIGNAL(doRenderStep()), &gameCore, SLOT(renderStep()));
  
  //connect(this, SIGNAL(doRenderStep()), &displayer, SLOT(requestReady()));
  //connect(&displayer, SIGNAL(ready( const Display& )), &gameCore, SLOT(renderStep( const Display& )));
  
  //connect(this, SIGNAL(doEventStep()), &controlInterface, SLOT(eventStep()));
  
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
  
  //emit requestGLContext();
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
void Runner::run()
{
  /* TODO, proper game loop goes here */
  qDebug() << "Runner::run";
  qDebug() << "currentContext: " << QGLContext::currentContext ();
  
  running = false;
  
  //gameInterface.initStep();
  
  while (!gameInterface.initialized()) {
    yieldCurrentThread ();
    usleep(50000);
  }
  running = true;
  
  /*while (running) {
    //qDebug() << "step " << i;
    //gameInterface.eventStep();
    gameInterface.logicStep();
    gameInterface.renderStep();
    usleep(50000);
  }*/
                                                                             
  const int TICKS_PER_SECOND = 60;                                           
  const int SKIP_TICKS = 1000000 / TICKS_PER_SECOND;                         
  const int MAX_FRAMESKIP = 5;                                               

  unsigned long long int next_game_tick = getTimeOfDay();
  int loops;                                             
  float interpolation;                                   

  bool game_is_running = true;
  while( game_is_running ) {  

    bool render = false;
    loops = 0;          
    while( getTimeOfDay() > next_game_tick && loops < MAX_FRAMESKIP) {
                                                                                       
      gameInterface.logicStep();                                        
                                                                        
      render = true;                                                    

      next_game_tick += SKIP_TICKS;
      loops++;                     
    }                              
    //qDebug() << "OUTER " << getTimeOfDay() << ":" << next_game_tick;
                                                                      
    if (render) {   
      gameInterface.renderStep();                                                    
      render = false;                                                 
    }                                                                 
    //usleep(1000);                                                     
    yieldCurrentThread ();                                          
                                                                      
                                                                      
    //interpolation = float( getTimeOfDay() + SKIP_TICKS - next_game_tick )
    //                / float( SKIP_TICKS );                               
    //display_game( interpolation );                                       
  }                                                  
  
  gameInterface.cleanUpStep();
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/
void Runner::normalQuit()
{
  /* TODO set the running variable to false or whatever :/ */
  running = false;
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



//==============================================================================
unsigned long long int Runner::getTimeOfDay()
{
        gettimeofday(&tv,0);
        return ((unsigned long long int)tv.tv_sec*1000000 + tv.tv_usec);
}


} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */

