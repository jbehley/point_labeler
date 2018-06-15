#include <gtest/gtest.h>
#include "data/misc.h"

using namespace glow;

std::ostream& operator<<(std::ostream& o, const Triangle& t) {
  o << t.i << "; " << t.j << "; " << t.k;

  return o;
}

TEST(TriangulateTest, triangulate) {
  std::vector<vec2> points;
  points.push_back(vec2(370, 213));
  points.push_back(vec2(369, 133));
  points.push_back(vec2(505, 172));
  points.push_back(vec2(485, 247));

  //  std::cout << "test1" << std::endl;
  std::vector<Triangle> tris;
  triangulate(points, tris);

  ASSERT_EQ(2, tris.size());

  //  std::cout << "test2" << std::endl;
  points.clear();
  points.push_back(vec2(191, 120));
  points.push_back(vec2(406, 129));
  points.push_back(vec2(506, 222));
  points.push_back(vec2(440, 407));
  points.push_back(vec2(329, 407));
  points.push_back(vec2(171, 322));

  tris.clear();
  triangulate(points, tris);
  //  std::cout << tris.size() << std::endl;
  //  for (auto t : tris) std::cout << t << std::endl;

  ASSERT_EQ(4, tris.size());
}
