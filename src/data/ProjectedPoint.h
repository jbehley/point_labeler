#ifndef SRC_DATA_PROJECTEDPOINT_H_
#define SRC_DATA_PROJECTEDPOINT_H_

#include <glow/glutil.h>
#include <stdint.h>

struct ProjectedPoint {
  glow::vec2 pos;
  uint32_t timestamp;
  uint32_t index;
};

#endif /* SRC_DATA_PROJECTEDPOINT_H_ */
