#ifndef SRC_DATA_QUADTREE_H_
#define SRC_DATA_QUADTREE_H_

#include "data/ProjectedPoint.h"

/** \brief
 *
 *  TODO: more generic implementation.
 *
 *  \author behley
 */

class QuadTree {
 public:
  /** \brief initialize quad tree with the projected points. **/
  void initialize(const std::vector<ProjectedPoint>& points);

  /** \brief get all radius neighbors or their indexes **/
  void radiusSearch(const glow::vec2& q, float radius, std::vector<uint32_t>& indexes);
};

#endif /* SRC_DATA_QUADTREE_H_ */
