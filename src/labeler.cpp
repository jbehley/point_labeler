#include <QtWidgets/QApplication>
#include "widget/Mainframe.h"

int main(int argc, char** argv) {
  QApplication app(argc, argv);

  Mainframe frame;
  frame.show();


  return app.exec();
}
