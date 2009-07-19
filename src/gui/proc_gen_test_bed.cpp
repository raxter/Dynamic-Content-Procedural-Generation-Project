/****************************************************************************
**
** For Copyright & Licensing information, see LICENSE in project root
**
****************************************************************************/


#include "proc_gen_test_bed.h"

namespace ProcGen {

namespace GUI {


/****************************************************************************
**
** Author: Richard Baxter
**
** Default Constructor
**
****************************************************************************/

ProcGenTestBed::ProcGenTestBed(QApplication &application) : QMainWindow(), application(application)
{
  setupUi(this);
  
  glDisplay = new GameComponent::GLDisplay();
  
  testGame = new GameComponent::TestGame();
  
  controlInterface = new AbstractGameComponent::ControlInterface();
  
  glDisplayWidget = new GameComponent::GLDisplayWidget(*testGame, *glDisplay, *controlInterface);
  this->setCentralWidget(glDisplayWidget);
  
  glDisplay->setGLDisplayWidget(glDisplayWidget);
  
  gameRunner = new AbstractGameComponent::Runner(*glDisplayWidget);
  
  gameRunner->runGame();
  glDisplayWidget->moveToThread(gameRunner);
}


/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/

ProcGenTestBed::~ProcGenTestBed() {
  delete glDisplayWidget;
  delete glDisplay;
  delete testGame;
  delete gameRunner;
}


} /* end of namespace GUI */

} /* end of namespace ProcGen */











