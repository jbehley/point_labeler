#include "label_utils.h"

#include <QtCore/QFile>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtXml/QDomDocument>

void getLabelNames(const std::string& filename, std::map<uint32_t, std::string>& label_names) {
  QDomDocument doc("mydocument");
  QFile file(QString::fromStdString(filename));
  if (!file.open(QIODevice::ReadOnly)) return;

  if (!doc.setContent(&file)) {
    file.close();
    return;
  }

  file.close();

  QDomElement docElem = doc.documentElement();
  QDomElement rootNode = doc.firstChildElement("config");

  QDomElement n = rootNode.firstChildElement("label");
  for (; !n.isNull(); n = n.nextSiblingElement("label")) {
    std::string name = n.firstChildElement("name").text().toStdString();
    uint32_t id = n.firstChildElement("id").text().toInt();

    label_names[id] = name;

    if (!n.firstChildElement("moving").isNull()) {
      label_names[n.firstChildElement("moving").text().toInt()] = name + " (moving)";
    }
  }
}

void getLabelColors(const std::string& filename, std::map<uint32_t, glow::GlColor>& label_colors) {
  QDomDocument doc("mydocument");
  QFile file(QString::fromStdString(filename));
  if (!file.open(QIODevice::ReadOnly)) return;

  if (!doc.setContent(&file)) {
    file.close();
    return;
  }

  file.close();

  QDomElement docElem = doc.documentElement();
  QDomElement rootNode = doc.firstChildElement("config");

  QDomElement n = rootNode.firstChildElement("label");
  for (; !n.isNull(); n = n.nextSiblingElement("label")) {
    std::string name = n.firstChildElement("name").text().toStdString();
    uint32_t id = n.firstChildElement("id").text().toInt();
    QString color_string = n.firstChildElement("color").text();
    QStringList tokens = color_string.split(" ");

    int32_t R = tokens.at(0).toInt();
    int32_t G = tokens.at(1).toInt();
    int32_t B = tokens.at(2).toInt();

    label_colors[id] = glow::GlColor::FromRGB(R, G, B);

    if (!n.firstChildElement("moving").isNull()) {
      label_colors[n.firstChildElement("moving").text().toInt()] = label_colors[id];
    }
  }
}

void getLabels(const std::string& filename, std::vector<Label>& labels) {
  QDomDocument doc("mydocument");
  QFile file(QString::fromStdString(filename));
  if (!file.open(QIODevice::ReadOnly)) return;

  if (!doc.setContent(&file)) {
    file.close();
    return;
  }

  file.close();

  QDomElement docElem = doc.documentElement();
  QDomElement rootNode = doc.firstChildElement("config");

  QDomElement n = rootNode.firstChildElement("label");
  for (; !n.isNull(); n = n.nextSiblingElement("label")) {
    Label label;

    label.name = n.firstChildElement("name").text().toStdString();
    label.id = n.firstChildElement("id").text().toInt();
    QString color_string = n.firstChildElement("color").text();
    QStringList tokens = color_string.split(" ");

    int32_t R = tokens.at(0).toInt();
    int32_t G = tokens.at(1).toInt();
    int32_t B = tokens.at(2).toInt();

    label.color = glow::GlColor::FromRGB(R, G, B);

    if (!n.firstChildElement("moving").isNull()) {
      label.potentiallyMoving = true;
      label.id_moving = n.firstChildElement("moving").text().toInt();
    }

    if (!n.firstChildElement("instanceable").isNull()) {
      if (n.firstChildElement("instanceable").text() == "true") {
        label.instanceable = true;
      }
    }

    label.rootCategory = n.firstChildElement("root").text().toStdString();
    labels.push_back(label);
  }
}
