/**
 * \headerfile misc.h
 *
 * \brief some functions.
 *
 * \author behley
 */

#ifndef MISC_H_
#define MISC_H_

#include <algorithm>
#include <string>
#include <vector>

#include <glow/glutil.h>

/** \brief remove whitespaces at the beginning and end of a string. **/
std::string trim(const std::string& str, const std::string& whitespaces = " \0\t\n\r\x0B");

/** \brief splits a string in tokens, which are seperated by delim. **/
std::vector<std::string> split(const std::string& line, const std::string& delim = " ");

template <typename T>
class IndexValue {
 public:
  IndexValue() : index(-1), value(nullptr) {}
  IndexValue(int32_t i, const T* v) : index(i), value(v) {}

  int32_t index;
  const T* value;

  friend bool operator<(const IndexValue<T>& l, const IndexValue<T>& r) { return (*l.value < *r.value); }
  friend bool operator==(const IndexValue<T>& l, const IndexValue<T>& r) { return (*l.value == *r.value); }
};

/** \brief determine indexes of set_difference between a and b. **/
template <typename T, typename A>
void index_difference(const std::vector<T, A>& a, const std::vector<T, A>& b, std::vector<uint32_t>& result) {
  std::vector<IndexValue<T>> proxy_a;
  std::vector<IndexValue<T>> proxy_b;
  std::vector<IndexValue<T>> proxy_result(a.size());

  for (int32_t i = 0; i < int32_t(a.size()); ++i) proxy_a.push_back(IndexValue<T>(i, &a[i]));
  for (int32_t i = 0; i < int32_t(b.size()); ++i) proxy_b.push_back(IndexValue<T>(i, &b[i]));

  std::sort(proxy_a.begin(), proxy_a.end());
  std::sort(proxy_b.begin(), proxy_b.end());

  auto end = std::set_difference(proxy_a.begin(), proxy_a.end(), proxy_b.begin(), proxy_b.end(), proxy_result.begin());

  result.clear();
  for (auto it = proxy_result.begin(); it != end; ++it) result.push_back(it->index);
}

struct Triangle {
  glow::vec2 i, j, k;
};

/** \brief test if point p inside triangle defined by v1, v2, v3 (counter-clockwise order) **/
bool insideTriangle(const glow::vec2& p, const glow::vec2& v1, const glow::vec2& v2, const glow::vec2& v3);

/** \brief perform triangulation of 2d points using ear cuttin algorithm **/
bool triangulate(const std::vector<glow::vec2>& points, std::vector<Triangle>& triangles);

/** \brief check if value is inside vector **/
template <typename T, typename A>
bool contains(const std::vector<T, A>& vector, const T& value) {
  for (const auto& v : vector) {
    if (v == value) return true;
  }

  return false;
}

#endif /* MISC_H_ */
