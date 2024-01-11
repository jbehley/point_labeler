
#include <QtWidgets/QApplication>
#include "widget/Mainframe.h"

#include <glow/GlCapabilities.h>

int main(int argc, char** argv) {
  QApplication app(argc, argv);

  Mainframe frame;
  frame.show();
  frame.resize(1200, 900);

  // QPalette pal = app.palette();
  // pal.setColor(QPalette::Window, Qt::black);
  // app.setPalette(pal);

//  std::cout << glow::GlCapabilities::getInstance() << std::endl;
  return app.exec();
}
