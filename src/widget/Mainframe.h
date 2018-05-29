/** \brief main widget with 4 viewports showing the point cloud from different perspectives. **/

#ifndef MAINFRAME_H_
#define MAINFRAME_H_

#include <QtWidgets/QMainWindow>
#include <QtCore/QSignalMapper>
#include "ui_MainFrame.h"
#include "data/geometry.h"
#include <stdint.h>
#include "LabelButton.h"

class Mainframe: public QMainWindow
{
  Q_OBJECT
  public:
    Mainframe();
    ~Mainframe();

  public slots:
    void open();
    void save();
    void changeRadius(int radius);
    void changeMode(int mode);
    void addNewCylinder();
    void removeCylinder();
    void cylinderFinished();
    void updateFiltering(bool value);
    void labelBtnReleased(QWidget*);
    void changeCylinderRadius(double radius);
    void changeCylinderLabel();
    void changeSelectedCylinder();
    void cylinderPointLabeling();

  protected:
    void readXYZ(const std::string& filename);
    void generateLabelButtons();
    void closeEvent(QCloseEvent *event);

    std::vector<Point3f> points;
    std::vector<uint32_t> labels;
    std::vector<Cylinder> cylinders;

    std::vector<uint32_t> filteredLabels;
    std::string filename;

  protected slots:
    void unsavedChanges();

  private:
    Ui::MainWindow ui;
    QSignalMapper* modeMapper, *labelButtonMapper, *radiusMapper;
    std::vector<LabelButton*> labelButtons;
    std::map<LabelButton*, uint32_t> labelIds;
    std::map<int32_t, uint32_t> idxLabelMap;
    std::map<uint32_t, int32_t> labelIdxMap;
    bool mChangesSinceLastSave;
    QString lastDirectory;

    Point3f midpoint;
};

#endif /* MAINFRAME_H_ */
