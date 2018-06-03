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
  }
}

void KittiReader::retrieve(uint32_t index, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
                           std::vector<LabelsPtr>& labels) {
  // TODO: only re-read point clouds that are needed.
  points.clear();
  labels.clear();

  // find nearby scans.
  Eigen::Vector4f midpoint = poses_[index].col(3);
  for (uint32_t t = 0; t < velodyne_filenames_.size(); ++t) {
    Eigen::Vector4f other_midpoint = poses_[t].col(3);
    if ((midpoint - other_midpoint).norm() < 100.0f) {
      points.push_back(std::shared_ptr<Laserscan>(new Laserscan));
      readPoints(velodyne_filenames_[t], *points.back());
      points.back()->pose = poses_[t];
    }
  }

  std::cout << points.size() << " point clouds read." << std::endl;

  // TODO: read also labels.
}

void KittiReader::update(const std::vector<uint32_t>& indexes, std::vector<LabelsPtr>& labels) {}

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

  float max_remission = 0;

  for (uint32_t i = 0; i < num_points; ++i) {
    points[i].x = values[4 * i];
    points[i].y = values[4 * i + 1];
    points[i].z = values[4 * i + 2];
    remissions[i] = values[4 * i + 3];
    max_remission = std::max(remissions[i], max_remission);
  }

  for (uint32_t i = 0; i < num_points; ++i) {
    remissions[i] /= max_remission;
  }
}

void KittiReader::readLabels(const std::string& filename, std::vector<uint32_t>& labels) {}

void KittiReader::readPoses(const std::string& filename, std::vector<Eigen::Matrix4f>& poses) {
  poses = KITTI::Odometry::loadPoses(filename);

  // convert from camera to velodyne coordinate system.
  Eigen::Matrix4f Tr = calib_["Tr"];
  Eigen::Matrix4f Tr_inv = Tr.inverse();
  for (uint32_t i = 0; i < poses.size(); ++i) {
    poses[i] = Tr_inv * poses[i] * Tr;
  }
}
