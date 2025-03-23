
#include <QtWidgets/QApplication>
#include "widget/Mainframe.h"

#include <glow/GlCapabilities.h>

#define FLAG_OPEN_DIR "--open-dir"

std::map<std::string, std::string> parseArgs(int argc, char** argv) {
  std::map<std::string, std::string> parsedArgs;

  for (int i = 1; i < argc; i++) {
    std::string flag = argv[i];
    if (flag == FLAG_OPEN_DIR) {
      if (argc > i) {
        parsedArgs[FLAG_OPEN_DIR] = argv[i++ + 1];
      }
    }
  }

  return parsedArgs;
}

int main(int argc, char** argv) {
  QApplication app(argc, argv);

  Mainframe frame;
  auto parsedArgs = parseArgs(argc, argv);

  if (parsedArgs.find(FLAG_OPEN_DIR) != parsedArgs.end()) {
    auto dir = parsedArgs[FLAG_OPEN_DIR];
    std::cout << "Opening dir: " << dir << std::endl;
    frame.open(QString::fromStdString(dir));
  }

  frame.show();
  frame.resize(1200, 900);
//  std::cout << glow::GlCapabilities::getInstance() << std::endl;

  return app.exec();
}
