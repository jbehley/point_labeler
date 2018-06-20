#ifndef SRC_WIDGET_COMMON_H_
#define SRC_WIDGET_COMMON_H_

#include <data/Pointcloud.h>
#include <memory>
#include "data/geometry.h"

typedef std::shared_ptr<Laserscan> PointcloudPtr;
typedef std::shared_ptr<std::vector<uint32_t>> LabelsPtr;
typedef std::shared_ptr<std::vector<float>> ColorsPtr;

#endif /* SRC_WIDGET_COMMON_H_ */
