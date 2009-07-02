#ifndef __PROCGEB_GAME_ABSTRACTGAME_H__
#define __PROCGEB_GAME_ABSTRACTGAME_H__

#include <QObject>

#include "abstract_display.h"

namespace ProcGen {

namespace Game {

  
class AbstractGame : public QObject {

  Q_OBJECT

  public: /* class specific */

  AbstractGame();
  virtual ~AbstractGame();
  
  public slots: /* virtual */

  virtual void initStep(void * pntr = 0);
  virtual void logicStep(void * pntr = 0);
  virtual void renderStep(void * pntr = 0);

  public: /* methods */
  void setDisplayer(AbstractDisplay* displayer);

  protected: /* variables */
  AbstractDisplay* displayer;
};

} /* end of namespace Game */

} /* end of namespace ProcGen */


#endif /* __PROCGEB_ABSTRACTGAME_GAME_H__ */

