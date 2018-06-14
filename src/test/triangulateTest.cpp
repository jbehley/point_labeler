#include <gtest/gtest.h>
#include "data/misc.h"

using namespace glow;

TEST(TriangulateTest, triangulate) {
  std::vector<vec2> points;
  points.push_back(vec2(370, 213));
  points.push_back(vec2(369, 133));
  points.push_back(vec2(505, 172));
  points.push_back(vec2(485, 247));

  std::vector<Triangle> tris;
  triangulate(points, tris);

  ASSERT_EQ(2, tris.size());
}
