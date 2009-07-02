/****************************************************************************
**
** For Copyright & Licensing information, see LICENSE in project root
**
****************************************************************************/

#ifndef __PROCGEN_PROCGENTESTBED_H__
#define __PROCGEN_PROCGENTESTBED_H__

#include <QMainWindow>
#include <QApplication>

#include "ui_proc_gen_test_bed.h"

#include "gl_display.h"
#include "game/test_game.h"
#include "game/game_runner.h"

namespace ProcGen {

namespace GUI {


class ProcGenTestBed : public QMainWindow, private Ui::ProcGenTestBed{

  Q_OBJECT
  
  public: /* class specific */

  ProcGenTestBed(QApplication &application);
  ~ProcGenTestBed();
  
  private: /* variables */

  QApplication &application;
  GLDisplayWidget* glDisplayWidget;
  GLDisplay* glDisplay;
  Game::TestGame* testGame;
  Game::GameRunner* gameRunner;
};

} /* end of namespace GUI */

} /* end of namespace ProcGen */


#endif /* __PROCGEN_PROCGENTESTBED_H__ */

