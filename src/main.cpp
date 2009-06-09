
#include <QApplication>

#include "gui/proc_gen_test_bed.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);

  ProcGen::GUI::ProcGenTestBed procGenTB(app);
  
  procGenTB.show();
  
  return app.exec();
}
