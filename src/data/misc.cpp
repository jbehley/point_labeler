#include "misc.h"

#include <stdint.h>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <iostream>

std::string trim(const std::string& str, const std::string& whitespaces) {
  int32_t beg = 0;
  int32_t end = 0;

  /** find the beginning **/
  for (beg = 0; beg < (int32_t)str.size(); ++beg) {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i) {
      if (str[beg] == whitespaces[i]) {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  /** find the end **/
  for (end = int32_t(str.size()) - 1; end > beg; --end) {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i) {
      if (str[end] == whitespaces[i]) {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  return str.substr(beg, end - beg + 1);
}

std::vector<std::string> split(const std::string& line, const std::string& delim) {
  QString string = QString::fromStdString(line);

  QStringList list = string.split(QString::fromStdString(delim));
  std::vector<std::string> tokens;

  QStringListIterator it(list);

  while (it.hasNext()) tokens.push_back(it.next().toStdString());

  return tokens;
}

bool insideTriangle(const glow::vec2& p, const glow::vec2& v1, const glow::vec2& v2, const glow::vec2& v3) {
  float b0 = ((v2.x - v1.x) * (v3.y - v1.y) - (v3.x - v1.x) * (v2.y - v1.y));

  if (std::abs(b0) > 0) {
    // compute barycentric coordinates.
    float b1 = (((v2.x - p.x) * (v3.y - p.y) - (v3.x - p.x) * (v2.y - p.y)) / b0);

    float b2 = (((v3.x - p.x) * (v1.y - p.y) - (v1.x - p.x) * (v3.y - p.y)) / b0);
    float b3 = 1.0f - b1 - b2;

    // only if all are greater equal 0, the point can be inside.
    return (b1 > 0) && (b2 > 0) && (b3 > 0);
  }

  return false;
}

bool triangulate(const std::vector<glow::vec2>& points, std::vector<Triangle>& triangles) {
  int32_t i = 0;
  int32_t lastear = -1;
  std::vector<glow::vec2> lst = points;

  glow::vec2 p1, p, p2;
  do {
    lastear = lastear + 1;

    // check next ear.
    p1 = lst[((i - 1) + lst.size()) % lst.size()];
    p = lst[(i + lst.size()) % lst.size()];
    p2 = lst[((i + 1) + lst.size()) % lst.size()];

    // is corner convex or concave?
    //    float l = ((p1.x - p.x) * (p2.y - p.y) - (p1.y - p.y) * (p2.x - p.x));

    float g = (p.y - p1.y) * (p2.x - p1.x) + (p1.x - p.x) * (p2.y - p1.y);
    // signed triangle area:
    //    float c = (p2.x - p1.x) * (p2.y + p1.y) + (p.x - p2.x) * (p.y + p2.y) + (p1.x - p.x) * (p1.y + p.y);

    if (g < 0) {
      // check for other point inside triangle.
      bool intriangle = false;
      for (int32_t j = 2; j <= int32_t(lst.size()) - 2; ++j) {
        glow::vec2 pt = lst[(i + j + lst.size()) % lst.size()];

        if (insideTriangle(pt, p1, p, p2)) {
          intriangle = true;
          break;
        }
      }

      // found an ear, remove vertex.
      if (!intriangle) {
        Triangle tri;
        tri.i = p1;
        tri.j = p;
        tri.k = p2;

        triangles.push_back(tri);
        lst.erase(lst.begin() + i);

        lastear = 0;

        i = i - 1;
      }
    }

    i = (i + 1) % lst.size();
  } while ((lastear < int32_t(lst.size()) * 2) && (int32_t(lst.size()) != 3));

  if (lst.size() == 3) {
    Triangle tri;
    tri.i = lst[0];
    tri.j = lst[1];
    tri.k = lst[2];

    triangles.push_back(tri);
  }

  return (lst.size() == 3);
}
