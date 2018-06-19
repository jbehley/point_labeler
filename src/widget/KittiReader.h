#ifndef SRC_WIDGET_KITTIREADER_H_
#define SRC_WIDGET_KITTIREADER_H_

#include <stdint.h>
#include <QtCore/QString>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include "common.h"
#include "data/kitti_utils.h"

/** \brief
 *
 *
 *
 *  \author behley
 */

class KittiReader {
 public:
  struct Tile {
    int32_t i, j;
    std::vector<uint32_t> indexes;
    float x, y, size;
  };

  void initialize(const QString& directory);

  uint32_t count() const { return velodyne_filenames_.size(); }

  void setMaximumDistance(float distance) { maxDistance_ = distance; }

  /** \brief get points and labels for given index. **/
  //  void retrieve(uint32_t index, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
  //                std::vector<LabelsPtr>& labels);

  void retrieve(const Eigen::Vector3f& position, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
                std::vector<LabelsPtr>& labels);

  void retrieve(uint32_t i, uint32_t j, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
                std::vector<LabelsPtr>& labels);

  void update(const std::vector<uint32_t>& indexes, std::vector<LabelsPtr>& labels);

  void setTileSize(float size);

  const std::vector<Tile>& getTiles() const { return tiles_; }

  const Tile& getTile(const Eigen::Vector3f& pos) const;
  const Tile& getTile(uint32_t i, uint32_t j) const;

  const Eigen::Vector2i& numTiles() const { return numTiles_; }

 protected:
  void readPoints(const std::string& filename, Laserscan& scan);
  void readLabels(const std::string& filename, std::vector<uint32_t>& labels);
  void readPoses(const std::string& filename, std::vector<Eigen::Matrix4f>& poses);

  KITTICalibration calib_;
  std::vector<Eigen::Matrix4f> poses_;
  std::vector<std::string> velodyne_filenames_;
  std::vector<std::string> label_filenames_;

  // cache reads from before.
  std::map<uint32_t, PointcloudPtr> pointsCache_;
  std::map<uint32_t, LabelsPtr> labelCache_;

  float maxDistance_{15.0f};

  inline uint32_t tileIdxToOffset(uint32_t i, uint32_t j) const { return i + j * numTiles_.x(); }

  float tileSize_{50};
  std::vector<Tile> tiles_;
  Eigen::Vector2f offset_;
  Eigen::Vector2i numTiles_;
};

#endif /* SRC_WIDGET_KITTIREADER_H_ */
