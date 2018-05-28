#include "misc.h"

#include <QtCore/QString>
#include <QtCore/QStringList>
#include <stdint.h>
#include <iostream>

std::string trim(const std::string& str, const std::string& whitespaces)
{
  int32_t beg = 0;
  int32_t end = 0;

  /** find the beginning **/
  for (beg = 0; beg < (int32_t) str.size(); ++beg)
  {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i)
    {
      if (str[beg] == whitespaces[i])
      {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  /** find the end **/
  for (end = int32_t(str.size()) - 1; end > beg; --end)
  {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i)
    {
      if (str[end] == whitespaces[i])
      {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  return str.substr(beg, end - beg + 1);
}

std::vector<std::string> split(const std::string& line,
    const std::string& delim)
{
  QString string = QString::fromStdString(line);

  QStringList list = string.split(QString::fromStdString(delim));
  std::vector<std::string> tokens;

  QStringListIterator it(list);

  while (it.hasNext())
    tokens.push_back(it.next().toStdString());

  return tokens;
}
