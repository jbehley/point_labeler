#include <stdint.h>
#include <widget/KittiReader.h>
#include <QtCore/QDir>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include "rv/string_utils.h"

void KittiReader::initialize(const QString& directory) {
  velodyne_filenames_.clear();

  QDir base_dir(directory);
  QDir velodyne_dir(base_dir.filePath("velodyne"));
  QStringList entries = velodyne_dir.entryList(QDir::Files, QDir::Name);
  for (int32_t i = 0; i < entries.size(); ++i) {
    velodyne_filenames_.push_back(velodyne_dir.filePath(entries.at(i)).toStdString());
  }

  if (!base_dir.exists("calib.txt"))
    throw std::runtime_error("Missing calibration file: " + base_dir.filePath("calib.txt").toStdString());

  calib_.initialize(base_dir.filePath("calib.txt").toStdString());

  readPoses(base_dir.filePath("poses.txt").toStdString(), poses_);

  // create label dir, etc.
  QDir labels_dir(base_dir.filePath("labels"));
  //  find corresponding label files.
  if (!labels_dir.exists()) base_dir.mkdir("labels");

  for (uint32_t i = 0; i < velodyne_filenames_.size(); ++i) {
    std::ifstream in(velodyne_filenames_[i].c_str());
    in.seekg(0, std::ios::end);
    uint32_t num_points = in.tellg() / (4 * sizeof(float));
    in.close();

    QString filename = QFileInfo(QString::fromStdString(velodyne_filenames_[i])).baseName() + ".label";
    if (!labels_dir.exists(filename)) {
      std::ofstream out(labels_dir.filePath(filename).toStdString().c_str());

      std::vector<uint32_t> labels(num_points, 0);
      out.write(reinterpret_cast<const char*>(labels.data()), num_points * sizeof(uint32_t));

      out.close();
    }

    label_filenames_.push_back(labels_dir.filePath(filename).toStdString());
  }
}

void KittiReader::retrieve(uint32_t index, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
                           std::vector<LabelsPtr>& labels) {
  indexes.clear();
  points.clear();
  labels.clear();

  std::vector<int32_t> indexesBefore;
  for (auto it = pointsCache_.begin(); it != pointsCache_.end(); ++it) indexesBefore.push_back(it->first);
  std::vector<int32_t> indexesAfter;

  uint32_t scansRead = 0;

  // find nearby scans.
  Eigen::Vector4f midpoint = poses_[index].col(3);
  for (uint32_t t = 0; t < velodyne_filenames_.size(); ++t) {
    Eigen::Vector4f other_midpoint = poses_[t].col(3);
    if ((midpoint - other_midpoint).norm() < maxDistance_) {
      indexesAfter.push_back(t);

      indexes.push_back(t);

      if (pointsCache_.find(t) == pointsCache_.end()) {
        scansRead += 1;

        points.push_back(std::shared_ptr<Laserscan>(new Laserscan));
        readPoints(velodyne_filenames_[t], *points.back());
        pointsCache_[t] = points.back();
        points.back()->pose = poses_[t];

        labels.push_back(std::shared_ptr<std::vector<uint32_t>>(new std::vector<uint32_t>()));
        readLabels(label_filenames_[t], *labels.back());
        labelCache_[t] = labels.back();

        if (points.back()->size() != labels.back()->size()) {
          std::cout << "Filename: " << velodyne_filenames_[t] << std::endl;
          std::cout << "Filename: " << label_filenames_[t] << std::endl;
          std::cout << "num. points = " << points.back()->size() << " vs. num. labels = " << labels.back()->size()
                    << std::endl;
          throw std::runtime_error("Inconsistent number of labels.");
        }

      } else {
        points.push_back(pointsCache_[t]);
        labels.push_back(labelCache_[t]);
      }
    }
  }

  std::cout << scansRead << " point clouds read." << std::endl;

  // FIXME: keep more scans in cache. not only remove unloaded scans.

  std::sort(indexesBefore.begin(), indexesBefore.end());
  std::sort(indexesAfter.begin(), indexesAfter.end());

  std::vector<int32_t> needsDelete(indexesBefore.size());
  std::vector<int32_t>::iterator end = std::set_difference(
      indexesBefore.begin(), indexesBefore.end(), indexesAfter.begin(), indexesAfter.end(), needsDelete.begin());

  for (auto it = needsDelete.begin(); it != end; ++it) {
    pointsCache_.erase(*it);
    labelCache_.erase(*it);
  }

  // my merge for reference:
  //  uint32_t i_before = 0;
  //  uint32_t i_after = 0;
  //  for (; i_before < indexesBefore.size(); ++i_before) {
  //    while (i_after < indexesAfter.size() && indexesAfter[i_after] < indexesBefore[i_before]) ++i_after;
  //    if (i_after == indexesAfter.size()) break;
  //    if (indexesAfter[i_after] == indexesBefore[i_before]) continue;
  //    // if indexes does not match (index after must larger then index before), remove points from before.
  //    pointsCache_.erase(indexesBefore[i_before]);
  //    labelCache_.erase(indexesBefore[i_before]);
  //  }
  //
  //  for (; i_before < indexesBefore.size(); ++i_before) {
  //    pointsCache_.erase(indexesBefore[i_before]);
  //    labelCache_.erase(indexesBefore[i_before]);
  //  }
}

void KittiReader::update(const std::vector<uint32_t>& indexes, std::vector<LabelsPtr>& labels) {
  for (uint32_t i = 0; i < indexes.size(); ++i) {
    std::ofstream out(label_filenames_[indexes[i]].c_str());
    out.write((const char*)&(*labels[i])[0], labels[i]->size() * sizeof(uint32_t));
    out.close();
  }
}

void KittiReader::readPoints(const std::string& filename, Laserscan& scan) {
  std::ifstream in(filename.c_str(), std::ios::binary);
  if (!in.is_open()) return;

  scan.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (4 * sizeof(float));
  in.seekg(0, std::ios::beg);

  std::vector<float> values(4 * num_points);
  in.read((char*)&values[0], 4 * num_points * sizeof(float));

  in.close();
  std::vector<Point3f>& points = scan.points;
  std::vector<float>& remissions = scan.remissions;

  points.resize(num_points);
  remissions.resize(num_points);

  for (uint32_t i = 0; i < num_points; ++i) {
    points[i].x = values[4 * i];
    points[i].y = values[4 * i + 1];
    points[i].z = values[4 * i + 2];
    remissions[i] = values[4 * i + 3];
  }
}

void KittiReader::readLabels(const std::string& filename, std::vector<uint32_t>& labels) {
  std::ifstream in(filename.c_str(), std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "Unable to open label file. " << std::endl;
    return;
  }

  labels.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (sizeof(uint32_t));
  in.seekg(0, std::ios::beg);

  labels.resize(num_points);
  in.read((char*)&labels[0], num_points * sizeof(uint32_t));

  in.close();
}

void KittiReader::readPoses(const std::string& filename, std::vector<Eigen::Matrix4f>& poses) {
  poses = KITTI::Odometry::loadPoses(filename);

  // convert from camera to velodyne coordinate system.
  Eigen::Matrix4f Tr = calib_["Tr"];
  Eigen::Matrix4f Tr_inv = Tr.inverse();
  for (uint32_t i = 0; i < poses.size(); ++i) {
    poses[i] = Tr_inv * poses[i] * Tr;
  }
}
