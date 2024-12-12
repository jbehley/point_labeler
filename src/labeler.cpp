
#include <QtWidgets/QApplication>
#include "widget/Mainframe.h"

#include <glow/GlCapabilities.h>

void handleArgs(int argc, char** argv, Mainframe* frame) {
  char* dir;
  bool dirSet = false;

  for (int i = 1; i < argc; i++) {
    std::string flag = argv[i];
    if (flag == "--open-dir") {
      std::cout << ">>> Found --open-dir";
      if (argc > i) {
        dir = argv[i++ + 1];
        std::cout << dir << " <<<" << std::endl;
        dirSet = true;
      }
    }
  }

  if (dirSet) {
    std::cout << "Opening dir: " << dir << std::endl;
    frame->open(dir);
  }
}

int main(int argc, char** argv) {
  QApplication app(argc, argv);

  Mainframe frame;
  handleArgs(argc, argv, &frame);

  frame.show();
  frame.resize(1200, 900);
//  std::cout << glow::GlCapabilities::getInstance() << std::endl;

  return app.exec();
}
