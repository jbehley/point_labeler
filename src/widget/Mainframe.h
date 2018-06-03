#ifndef MAINFRAME_H_
#define MAINFRAME_H_

#include <stdint.h>
#include <QtCore/QSignalMapper>
#include <QtWidgets/QMainWindow>
#include "LabelButton.h"
#include "data/geometry.h"
#include "data/transform.h"
#include "ui_MainFrame.h"
#include "common.h"
#include "KittiReader.h"

// TODO: undo.


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

  void generateLabelButtons();
  void closeEvent(QCloseEvent* event);



  std::vector<uint32_t> indexes_;
  std::vector<PointcloudPtr> points_;
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

  KittiReader reader_;
};

#endif /* MAINFRAME_H_ */
