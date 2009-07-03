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

#include "game_components/control_interface.h"
#include "game_components/gl_display.h"
#include "game_components/test_game.h"
#include "abstract_game_components/runner.h"

namespace ProcGen {

namespace GUI {


class ProcGenTestBed : public QMainWindow, private Ui::ProcGenTestBed{

  Q_OBJECT
  
  public: /* class specific */

  ProcGenTestBed(QApplication &application);
  ~ProcGenTestBed();
  
  private: /* variables */

  QApplication &application;
  GameComponent::GLDisplayWidget* glDisplayWidget;
  GameComponent::GLDisplay* glDisplay;
  GameComponent::TestGame* testGame;
  GameComponent::ControlInterface* controlInterface;
  
  AbstractGameComponent::Runner* gameRunner;
};

} /* end of namespace GUI */

} /* end of namespace ProcGen */


#endif /* __PROCGEN_PROCGENTESTBED_H__ */

