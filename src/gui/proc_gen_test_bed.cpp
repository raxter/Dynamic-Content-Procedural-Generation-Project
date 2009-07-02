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
  
  glDisplayWidget = new GLDisplayWidget();
  this->setCentralWidget(glDisplayWidget);
  
  glDisplay = new GLDisplay(*glDisplayWidget);
  
  testGame = new Game::TestGame();
  gameRunner = new Game::GameRunner(*testGame, *glDisplay);
  
  gameRunner->runGame();
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











