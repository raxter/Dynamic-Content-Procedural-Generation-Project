#ifndef __PROCGEB_ABSTRACTGAMECOMPONENT_GAME_H__
#define __PROCGEB_ABSTRACTGAMECOMPONENT_GAME_H__

#include <QObject>

#include "display.h"

namespace ProcGen {

namespace AbstractGameComponent {

  
class Game : public QObject {

  Q_OBJECT

  public: /* class specific */

  Game();
  virtual ~Game();
  
  public slots: /* virtual */

  virtual void initStep(void * pntr = 0);
  virtual void logicStep(void * pntr = 0);
  virtual void renderStep(void * pntr = 0);

  public: /* methods */
  void setDisplayer(Display* displayer);

  protected: /* variables */
  Display* displayer;
};

} /* end of namespace AbstractGameComponent */

} /* end of namespace ProcGen */


#endif /* __PROCGEB_ABSTRACTGAME_GAME_H__ */

