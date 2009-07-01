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
  glDisplay = new GLDisplay();
  this->setCentralWidget(glDisplay);
}

/****************************************************************************
**
** Author: Richard Baxter
**
****************************************************************************/

ProcGenTestBed::~ProcGenTestBed() {
  delete glDisplay;
}


} /* end of namespace GUI */

} /* end of namespace ProcGen */











