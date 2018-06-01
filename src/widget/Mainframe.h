#ifndef MAINFRAME_H_
#define MAINFRAME_H_

#include <stdint.h>
#include <QtCore/QSignalMapper>
#include <QtWidgets/QMainWindow>
#include "LabelButton.h"
#include "data/geometry.h"
#include "data/transform.h"
#include "ui_MainFrame.h"

#include <memory>

typedef std::shared_ptr<std::vector<Point3f>> PointCloudPtr;
typedef std::shared_ptr<std::vector<uint32_t>> LabelsPtr;

/** \brief main widget showing the point cloud and tools to label a point cloud/multiple point clouds. **/
class Mainframe : public QMainWindow {
  Q_OBJECT
 public:
  Mainframe();
  ~Mainframe();

 public slots:
  void open();
  void save();
  void changeRadius(int radius);
  void changeMode(int mode);

  void updateFiltering(bool value);
  void labelBtnReleased(QWidget*);

 protected:
  /** \brief set current scan and depending on mode the corresponding points of the viewport. **/
  void setCurrentScanIdx(int32_t idx);

  //  void readXYZ(const std::string& filename);

  void openKITTI(const std::string& directory);

  void generateLabelButtons();
  void closeEvent(QCloseEvent* event);

  std::vector<Eigen::Matrix4f> poses_;
  std::vector<std::string> velodyne_filenames_;

  //    std::vector<Point3f> points;
  //    std::vector<uint32_t> labels;

  std::vector<PointCloudPtr> points_;
  std::vector<LabelsPtr> labels_;

  std::vector<uint32_t> filteredLabels;
  std::string filename;

 protected slots:
  void unsavedChanges();

 private:
  Ui::MainWindow ui;
  QSignalMapper* labelButtonMapper;
  std::vector<LabelButton*> labelButtons;
  std::map<LabelButton*, uint32_t> labelIds;
  std::map<int32_t, uint32_t> idxLabelMap;
  std::map<uint32_t, int32_t> labelIdxMap;
  bool mChangesSinceLastSave;
  QString lastDirectory;

  Point3f midpoint;
};

#endif /* MAINFRAME_H_ */
