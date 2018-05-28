#include "Mainframe.h"

#include <fstream>
#include <iostream>
#include <map>

#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtGui/QFileDialog>

#include "../data/misc.h"
#include "../data/label_utils.h"

#include "../data/geometry.h"
#include "../data/transform.h"

#include <QtGui/QMessageBox>

Mainframe::Mainframe() :
  mChangesSinceLastSave(false)
{
  ui.setupUi(this);

  connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(open()));
  connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(save()));

  /** initialize the paint button mapping **/
  modeMapper = new QSignalMapper(this);
  modeMapper->setMapping(ui.actionPaintMode, Viewport::PAINT);
  modeMapper->setMapping(ui.actionCylinderMode, Viewport::CYLINDER);

  connect(ui.actionPaintMode, SIGNAL(triggered()), modeMapper, SLOT(map()));
  connect(ui.actionCylinderMode, SIGNAL(triggered()), modeMapper, SLOT(map()));
  connect(modeMapper, SIGNAL(mapped(int)), this, SLOT(changeMode(int)));

  connect(ui.btnNewCylinder, SIGNAL(released()), this, SLOT(addNewCylinder()));
  connect(ui.btnRemoveCylinder, SIGNAL(released()), this,
      SLOT(removeCylinder()));

  ui.mViewportX->setFixedAxis(Viewport::X);
  ui.mViewportY->setFixedAxis(Viewport::Y);
  ui.mViewportZ->setFixedAxis(Viewport::Z);

  connect(ui.mViewportX, SIGNAL(newCylinderFinished()), this,
      SLOT(cylinderFinished()));
  connect(ui.mViewportY, SIGNAL(newCylinderFinished()), this,
      SLOT(cylinderFinished()));
  connect(ui.mViewportZ, SIGNAL(newCylinderFinished()), this,
      SLOT(cylinderFinished()));
  connect(ui.mViewportXYZ, SIGNAL(newCylinderFinished()), this,
      SLOT(cylinderFinished()));

  connect(ui.mViewportX, SIGNAL(cylinderChanged()), ui.mViewportY,
      SLOT(updateGL()));
  connect(ui.mViewportX, SIGNAL(cylinderChanged()), ui.mViewportZ,
      SLOT(updateGL()));
  connect(ui.mViewportX, SIGNAL(cylinderChanged()), ui.mViewportXYZ,
      SLOT(updateGL()));

  connect(ui.mViewportY, SIGNAL(cylinderChanged()), ui.mViewportX,
      SLOT(updateGL()));
  connect(ui.mViewportY, SIGNAL(cylinderChanged()), ui.mViewportZ,
      SLOT(updateGL()));
  connect(ui.mViewportY, SIGNAL(cylinderChanged()), ui.mViewportXYZ,
      SLOT(updateGL()));

  connect(ui.mViewportZ, SIGNAL(cylinderChanged()), ui.mViewportY,
      SLOT(updateGL()));
  connect(ui.mViewportZ, SIGNAL(cylinderChanged()), ui.mViewportX,
      SLOT(updateGL()));
  connect(ui.mViewportZ, SIGNAL(cylinderChanged()), ui.mViewportXYZ,
      SLOT(updateGL()));

  connect(ui.mViewportX, SIGNAL(labelingChanged()), ui.mViewportY,
      SLOT(updateGL()));
  connect(ui.mViewportX, SIGNAL(labelingChanged()), ui.mViewportZ,
      SLOT(updateGL()));
  connect(ui.mViewportX, SIGNAL(labelingChanged()), ui.mViewportXYZ,
      SLOT(updateGL()));

  connect(ui.mViewportY, SIGNAL(labelingChanged()), ui.mViewportX,
      SLOT(updateGL()));
  connect(ui.mViewportY, SIGNAL(labelingChanged()), ui.mViewportZ,
      SLOT(updateGL()));
  connect(ui.mViewportY, SIGNAL(labelingChanged()), ui.mViewportXYZ,
      SLOT(updateGL()));

  connect(ui.mViewportZ, SIGNAL(labelingChanged()), ui.mViewportY,
      SLOT(updateGL()));
  connect(ui.mViewportZ, SIGNAL(labelingChanged()), ui.mViewportX,
      SLOT(updateGL()));
  connect(ui.mViewportZ, SIGNAL(labelingChanged()), ui.mViewportXYZ,
      SLOT(updateGL()));

  connect(ui.mViewportXYZ, SIGNAL(labelingChanged()), ui.mViewportX,
      SLOT(updateGL()));
  connect(ui.mViewportXYZ, SIGNAL(labelingChanged()), ui.mViewportY,
      SLOT(updateGL()));
  connect(ui.mViewportXYZ, SIGNAL(labelingChanged()), ui.mViewportZ,
      SLOT(updateGL()));

  connect(ui.mViewportX, SIGNAL(newCylinderFinished()), this,
      SLOT(unsavedChanges()));
  connect(ui.mViewportY, SIGNAL(newCylinderFinished()), this,
      SLOT(unsavedChanges()));
  connect(ui.mViewportZ, SIGNAL(newCylinderFinished()), this,
      SLOT(unsavedChanges()));
  connect(ui.mViewportX, SIGNAL(cylinderChanged()), this,
      SLOT(unsavedChanges()));
  connect(ui.mViewportY, SIGNAL(cylinderChanged()), this,
      SLOT(unsavedChanges()));
  connect(ui.mViewportZ, SIGNAL(cylinderChanged()), this,
      SLOT(unsavedChanges()));

  connect(ui.mViewportX, SIGNAL(labelingChanged()), this,
      SLOT(unsavedChanges()));
  connect(ui.mViewportY, SIGNAL(labelingChanged()), this,
      SLOT(unsavedChanges()));
  connect(ui.mViewportZ, SIGNAL(labelingChanged()), this,
      SLOT(unsavedChanges()));
  connect(ui.mViewportXYZ, SIGNAL(labelingChanged()), this,
      SLOT(unsavedChanges()));

  connect(ui.chkFilterLabels, SIGNAL(toggled(bool)), this,
      SLOT(updateFiltering(bool)));
  connect(ui.btnOverwrite, SIGNAL(toggled(bool)), ui.mViewportX,
      SLOT(setOverwrite(bool)));
  connect(ui.btnOverwrite, SIGNAL(toggled(bool)), ui.mViewportY,
      SLOT(setOverwrite(bool)));
  connect(ui.btnOverwrite, SIGNAL(toggled(bool)), ui.mViewportZ,
      SLOT(setOverwrite(bool)));
  connect(ui.btnOverwrite, SIGNAL(toggled(bool)), ui.mViewportXYZ,
      SLOT(setOverwrite(bool)));

  connect(ui.spinCylinderRadius, SIGNAL(valueChanged(double)), this,
      SLOT(changeCylinderRadius(double)));
  connect(ui.cmbCylinderLabel, SIGNAL(currentIndexChanged(int)), this,
      SLOT(changeCylinderLabel()));

  connect(ui.spinPointSize, SIGNAL(valueChanged(int)), ui.mViewportX,
      SLOT(setPointSize(int)));
  connect(ui.spinPointSize, SIGNAL(valueChanged(int)), ui.mViewportY,
      SLOT(setPointSize(int)));
  connect(ui.spinPointSize, SIGNAL(valueChanged(int)), ui.mViewportZ,
      SLOT(setPointSize(int)));
  connect(ui.spinPointSize, SIGNAL(valueChanged(int)), ui.mViewportXYZ,
      SLOT(setPointSize(int)));

  connect(ui.btnCylinderPointLabing, SIGNAL(released()), this,
      SLOT(cylinderPointLabeling()));

  radiusMapper = new QSignalMapper(this);
  radiusMapper->setMapping(ui.btnRadius5, 5);
  radiusMapper->setMapping(ui.btnRadius10, 10);
  radiusMapper->setMapping(ui.btnRadius20, 20);
  connect(ui.btnRadius5, SIGNAL(released()), radiusMapper, SLOT(map()));
  connect(ui.btnRadius10, SIGNAL(released()), radiusMapper, SLOT(map()));
  connect(ui.btnRadius20, SIGNAL(released()), radiusMapper, SLOT(map()));

  connect(radiusMapper, SIGNAL(mapped(int)), this, SLOT(changeRadius(int)));
  connect(ui.mRadiusSlider, SIGNAL(valueChanged(int)), this,
      SLOT(changeRadius(int)));
  connect(ui.lstCylinders, SIGNAL(itemSelectionChanged()), this,
      SLOT(changeSelectedCylinder()));

  /** load labels and colors **/
  std::map<uint32_t, std::string> label_names;
  std::map<uint32_t, ColorGL> label_colors;

  getLabelNames("labels.xml", label_names);
  getLabelColors("labels.xml", label_colors);

  ui.mViewportX->setLabelColors(label_colors);
  ui.mViewportY->setLabelColors(label_colors);
  ui.mViewportZ->setLabelColors(label_colors);
  ui.mViewportXYZ->setLabelColors(label_colors);

  generateLabelButtons();
}

Mainframe::~Mainframe()
{

}

void Mainframe::closeEvent(QCloseEvent *event)
{
  if (mChangesSinceLastSave)
  {
    int ret = QMessageBox::warning(this, tr("Unsaved changes."),
        tr("The annotation has been modified.\n"
          "Do you want to save your changes?"),
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
        QMessageBox::Save);
    if (ret == QMessageBox::Save)
      save();
    else if (ret == QMessageBox::Cancel)
    {
      event->ignore();
      return;
    }
  }

  event->accept();
}

void Mainframe::open()
{
  if (mChangesSinceLastSave)
  {
    int ret = QMessageBox::warning(this, tr("Unsaved changes."),
        tr("The annotation has been modified.\n"
          "Do you want to save your changes?"),
        QMessageBox::Save | QMessageBox::Cancel | QMessageBox::Discard,
        QMessageBox::Save);
    if (ret == QMessageBox::Save)
      save();
    else if (ret == QMessageBox::Cancel) return;
  }

  QString retValue = QFileDialog::getOpenFileName(this, "Select Logfile",
      lastDirectory, "XYZ logfiles(*.xyz)");

  if (!retValue.isNull())
  {
    if (retValue.endsWith(".xyz"))
    {
      QFileInfo info(retValue);
      lastDirectory = info.absolutePath();

      filename = retValue.toStdString();
      points.clear();
      labels.clear();
      cylinders.clear();
      ui.lstCylinders->clear();
      midpoint = Point3f(0, 0, 0);

      readXYZ(filename);

      changeMode(Viewport::NONE);

      ui.mViewportX->setPoints(points, labels);
      ui.mViewportY->setPoints(points, labels);
      ui.mViewportZ->setPoints(points, labels);
      ui.mViewportXYZ->setPoints(points, labels);

      for (uint32_t i = 0; i < cylinders.size(); ++i)
      {
        QString itemName = "Cylinder ";
        itemName += QString::number(i);
        ui.lstCylinders->addItem(itemName);
      }
      ui.lstCylinders->setCurrentRow(-1);

      ui.mViewportX->setCylinders(cylinders);
      ui.mViewportY->setCylinders(cylinders);
      ui.mViewportZ->setCylinders(cylinders);
      ui.mViewportXYZ->setCylinders(cylinders);
      QString title = "Point Labeler - ";
      title += QFileInfo(retValue).completeBaseName();
      setWindowTitle(title);

      mChangesSinceLastSave = false;
    }
    else
      return;

  }
}

void Mainframe::save()
{
  QFileInfo info(QString::fromStdString(filename));
  QString labelName = info.absolutePath() + "/" + info.baseName();

  std::string labels_file = labelName.toStdString();
  labels_file += ".label.xyz";

  //  std::string cylinder_file = RoSe::FileUtil::stripExtension(filename, 1);
  //  cylinder_file += ".cylinder.xyz";

  /* (3) overwrite the complete label file with new label information. */
  std::ofstream out(labels_file.c_str());
  for (uint32_t i = 0; i < points.size() - 1; ++i)
    out << labels[i] << std::endl;
  out << labels[labels.size() - 1];
  out.close();

  /* (4) write cylinder information to file **/
  //  out.open(cylinder_file.c_str());
  //  for (uint32_t i = 0; i < cylinders.size() - 1; ++i)
  //  {
  //    out << (cylinders[i].s.x + midpoint.x) << "," << (cylinders[i].s.y
  //        + midpoint.y) << "," << (cylinders[i].s.z + midpoint.z) << ",";
  //    out << (cylinders[i].e.x + midpoint.x) << "," << (cylinders[i].e.y
  //        + midpoint.y) << "," << (cylinders[i].e.z + midpoint.z) << ",";
  //    out << cylinders[i].radius << "," << cylinders[i].label << std::endl;
  //  }
  //
  //  out << (cylinders[cylinders.size() - 1].s.x + midpoint.x) << ","
  //      << (cylinders[cylinders.size() - 1].s.y + midpoint.y) << ","
  //      << (cylinders[cylinders.size() - 1].s.z + midpoint.z) << ",";
  //  out << (cylinders[cylinders.size() - 1].e.x + midpoint.x) << ","
  //      << (cylinders[cylinders.size() - 1].e.y + midpoint.y) << ","
  //      << (cylinders[cylinders.size() - 1].e.z + midpoint.z) << ",";
  //  out << cylinders[cylinders.size() - 1].radius << ","
  //      << cylinders[cylinders.size() - 1].label;
  //
  //  out.close();

  mChangesSinceLastSave = false;
}

void Mainframe::changeRadius(int value)
{
  ui.btnRadius5->setChecked(false);
  ui.btnRadius10->setChecked(false);
  ui.btnRadius20->setChecked(false);

  switch (value)
  {
    case 5:
      ui.btnRadius5->setChecked(true);
      break;
    case 10:
      ui.btnRadius10->setChecked(true);
      break;
    case 20:
      ui.btnRadius20->setChecked(true);
      break;
  }

  ui.mRadiusSlider->setValue(value);

  ui.mViewportX->setRadius(value);
  ui.mViewportY->setRadius(value);
  ui.mViewportZ->setRadius(value);
  ui.mViewportXYZ->setRadius(value);
}

void Mainframe::changeMode(int mode)
{
  ui.mViewportX->setMode(Viewport::NONE);
  ui.mViewportY->setMode(Viewport::NONE);
  ui.mViewportZ->setMode(Viewport::NONE);
  ui.mViewportXYZ->setMode(Viewport::NONE);

  ui.btnNewCylinder->setEnabled(false);
  ui.btnRemoveCylinder->setEnabled(false);
  ui.lstCylinders->setEnabled(false);

  if (mode == Viewport::PAINT && ui.actionPaintMode->isChecked()) /** PAINTMODE **/
  {
    ui.mViewportX->setMode(Viewport::PAINT);
    ui.mViewportY->setMode(Viewport::PAINT);
    ui.mViewportZ->setMode(Viewport::PAINT);
    ui.mViewportXYZ->setMode(Viewport::PAINT);

    ui.actionCylinderMode->setChecked(false);

    ui.mTools->setCurrentIndex(1);
  }
  else if (mode == Viewport::CYLINDER && ui.actionCylinderMode->isChecked())
  {
    ui.mViewportX->setMode(Viewport::CYLINDER);
    ui.mViewportY->setMode(Viewport::CYLINDER);
    ui.mViewportZ->setMode(Viewport::CYLINDER);
    ui.mViewportXYZ->setMode(Viewport::CYLINDER);

    ui.lstCylinders->setEnabled(true);
    ui.btnNewCylinder->setEnabled(true);
    if (ui.lstCylinders->count() > 0) ui.btnRemoveCylinder->setEnabled(true);

    ui.actionPaintMode->setChecked(false);

    ui.mTools->setCurrentIndex(0);
  }
  else if (mode == Viewport::NONE)
  {
    ui.actionPaintMode->setChecked(false);
    ui.actionCylinderMode->setChecked(false);
  }
}

void Mainframe::changeCylinderRadius(double radius)
{
  int32_t cylinderIdx = ui.lstCylinders->currentRow();
  if (cylinderIdx > -1)
  {
    cylinders[cylinderIdx].radius = radius;

    ui.mViewportX->updateGL();
    ui.mViewportY->updateGL();
    ui.mViewportZ->updateGL();
    ui.mViewportXYZ->updateGL();

    mChangesSinceLastSave = true;
  }
}

void Mainframe::changeCylinderLabel()
{
  int32_t cylinderIdx = ui.lstCylinders->currentRow();
  if (cylinderIdx > -1)
  {

    cylinders[cylinderIdx].label
        = idxLabelMap[ui.cmbCylinderLabel->currentIndex()];

    ui.mViewportX->updateGL();
    ui.mViewportY->updateGL();
    ui.mViewportZ->updateGL();
    ui.mViewportXYZ->updateGL();

    mChangesSinceLastSave = true;
  }
}

void Mainframe::changeSelectedCylinder()
{
  int32_t cylinderIdx = ui.lstCylinders->currentRow();
  if (cylinderIdx > -1)
  {
    ui.spinCylinderRadius->setValue(cylinders[cylinderIdx].radius);
    ui.cmbCylinderLabel->setCurrentIndex(
        labelIdxMap[cylinders[cylinderIdx].label]);

    ui.mViewportX->setSelectedCylinder(&cylinders[cylinderIdx], false);
    ui.mViewportY->setSelectedCylinder(&cylinders[cylinderIdx], false);
    ui.mViewportZ->setSelectedCylinder(&cylinders[cylinderIdx], false);
    ui.mViewportXYZ->setSelectedCylinder(&cylinders[cylinderIdx], false);
  }
}

void Mainframe::cylinderPointLabeling()
{
  std::cout << "labeling points inside cylinder" << std::endl;

  int32_t cylinderIdx = ui.lstCylinders->currentRow();
  if (cylinderIdx > -1)
  {
    const Point3f& s = cylinders[cylinderIdx].s;
    const Point3f& e = cylinders[cylinderIdx].e;
    const float radius = cylinders[cylinderIdx].radius;
    const uint32_t label = cylinders[cylinderIdx].label;

    /** first transform points in local coordinate system. **/
    Vector3f xAxis(1.0f, 0.0f, 0.0f);
    Vector3f yAxis(0.0f, 1.0f, 1.0f);

    Vector3f z = e - s;
    float length = z.Length();

    z = Normalize(z);
    Vector3f y = Cross(z, xAxis);
    if (y.Length() < 0.0001) y = Cross(z, yAxis);
    y = Normalize(y);
    Vector3f x = Cross(z, y);
    x = Normalize(x);

    Matrix4x4 m;
    m(0, 0) = x.x;
    m(1, 0) = x.y;
    m(2, 0) = x.z;
    m(0, 1) = y.x;
    m(1, 1) = y.y;
    m(2, 1) = y.z;
    m(0, 2) = z.x;
    m(1, 2) = z.y;
    m(2, 2) = z.z;
    m(0, 3) = 0;
    m(1, 3) = 0;
    m(2, 3) = 0;

    Matrix4x4 minv = Transpose(m);
    minv(0, 3) = 0;
    minv(1, 3) = 0;
    minv(2, 3) = 0;

    Transform t(m, minv);
    bool overwrite = ui.btnOverwrite->isChecked();
    for (uint32_t i = 0; i < points.size(); ++i)
    {
      Point3f p(points[i].x - s.x, points[i].y - s.y, points[i].z - s.z);
      Point3f tp = Inverse(t)(p);
      bool found = false;

      for (uint32_t j = 0; j < filteredLabels.size(); ++j)
        if (filteredLabels[j] == labels[i])
        {
          found = true;
          break;
        }

      if (!found && filteredLabels.size() > 0) continue;
      if (tp.z < 0.0 || tp.z > length) continue;
      double xy = std::sqrt(tp.x * tp.x + tp.y * tp.y);
      if (xy > radius) continue;
      if ((labels[i] == 0) || overwrite)
      {
        labels[i] = label;
      }
    }

    ui.mViewportX->updateGL();
    ui.mViewportY->updateGL();
    ui.mViewportZ->updateGL();
    ui.mViewportXYZ->updateGL();

    mChangesSinceLastSave = true;
  }
}

void Mainframe::readXYZ(const std::string& filename)
{
  std::ifstream in(filename.c_str());
  if (!in.is_open()) throw "unable to open XYZ file.";

  QFileInfo info(QString::fromStdString(filename));
  QString labelName = info.absolutePath() + "/" + info.baseName();
  std::string labels_file = labelName.toStdString();

  labels_file += ".label.xyz";

  //  std::string cylinder_file = RoSe::FileUtil::stripExtension(filename, 1);
  //  cylinder_file += ".cylinder.xyz";

  std::cout << "labels-filename: " << labels_file << std::endl;
  //  std::cout << "cylinder_filename: " << cylinder_file << std::endl;

  std::ifstream in_labels(labels_file.c_str());
  std::ofstream out_labels;

  bool generate_labels = false;
  if (!in_labels.is_open())
  {
    std::cout << "labels file not found. generating labels file." << std::endl;
    generate_labels = true;
    out_labels.open(labels_file.c_str());
    if (!out_labels.is_open()) throw "unable to generate label filename.";
  }

  //  std::ifstream in_cylinders(cylinder_file.c_str());

  std::string line;
  //  Point3f midpoint;

  in.peek();
  int idx = 0;
  while (!in.eof() && in.good())
  {
    std::getline(in, line);
    std::vector<std::string> tokens = split(line, ";");

    if (tokens.size() < 3) continue;

    float x = QString::fromStdString(tokens[0]).toFloat();
    float y = QString::fromStdString(tokens[1]).toFloat();
    float z = QString::fromStdString(tokens[2]).toFloat();

    if (!generate_labels)
    {
      std::getline(in_labels, line);
      labels.push_back(QString::fromStdString(line).toInt());
    }
    else
    {
      out_labels << "0" << std::endl;
      labels.push_back(0);
    }

    points.push_back(Point3f(x, y, z));
    midpoint += points.back();

    in.peek();
    ++idx;
  }

  midpoint /= points.size();

  /** substract the midpoint **/
  for (uint32_t i = 0; i < points.size(); ++i)
  {
    points[i].x -= midpoint.x;
    points[i].y -= midpoint.y;
    points[i].z -= midpoint.z;
  }

  //  if (in_cylinders.is_open())
  //  {
  //    std::string line;
  //    in_cylinders.peek();
  //    while (!in_cylinders.eof())
  //    {
  //      Cylinder c;
  //      std::getline(in_cylinders, line);
  //      std::vector<std::string> tokens = split(line, ",");
  //      if (tokens.size() < 8) continue; /** skip cylinder description without enough points **/
  //      c.s.x = RoSe::StringCast<float>(tokens[0]) - midpoint.x;
  //      c.s.y = RoSe::StringCast<float>(tokens[1]) - midpoint.y;
  //      c.s.z = RoSe::StringCast<float>(tokens[2]) - midpoint.z;
  //      c.startPointInitialized = true;
  //
  //      c.e.x = RoSe::StringCast<float>(tokens[3]) - midpoint.x;
  //      c.e.y = RoSe::StringCast<float>(tokens[4]) - midpoint.y;
  //      c.e.z = RoSe::StringCast<float>(tokens[5]) - midpoint.z;
  //      c.endPointInitialized = true;
  //
  //      c.radius = RoSe::StringCast<float>(tokens[6]);
  //      c.label = RoSe::StringCast<uint32_t>(tokens[7]);
  //
  //      cylinders.push_back(c);
  //    }
  //  }

  in.close();
  in_labels.close();
  //  in_cylinders.close();
  out_labels.close();
}

void Mainframe::addNewCylinder()
{
  cylinders.push_back(Cylinder());
  ui.mViewportX->setSelectedCylinder(&cylinders.back(), true);
  ui.mViewportY->setSelectedCylinder(&cylinders.back(), true);
  ui.mViewportZ->setSelectedCylinder(&cylinders.back(), true);
  ui.mViewportXYZ->setSelectedCylinder(&cylinders.back(), true);

  QString itemName = "Cylinder ";
  itemName += QString::number(cylinders.size());
  ui.lstCylinders->addItem(itemName);
  //  ui.lstCylinders->setCurrentRow(ui.lstCylinders->count() - 1);
  ui.lstCylinders->setEnabled(false);
  ui.btnNewCylinder->setEnabled(false);
  ui.btnRemoveCylinder->setEnabled(false);

  mChangesSinceLastSave = true;
}

void Mainframe::removeCylinder()
{
  int32_t cylinderIdx = ui.lstCylinders->currentRow();
  if (cylinderIdx < 0) return;

  ui.lstCylinders->clear();

  std::vector<Cylinder> tempCylinder;
  tempCylinder.reserve(cylinders.size() - 1);
  for (int32_t i = 0; i < (int32_t) cylinders.size(); ++i)
  {
    if (cylinderIdx != i)
    {
      tempCylinder.push_back(cylinders[i]);
      QString itemName = "Cylinder ";
      itemName += QString::number(tempCylinder.size());
      ui.lstCylinders->addItem(itemName);
    }
  }

  cylinders = tempCylinder;
  if (cylinders.size() == 0) ui.btnRemoveCylinder->setEnabled(false);

  ui.lstCylinders->setCurrentRow(-1);
  ui.mViewportX->updateGL();
  ui.mViewportY->updateGL();
  ui.mViewportZ->updateGL();
  ui.mViewportXYZ->updateGL();

  mChangesSinceLastSave = true;
}

void Mainframe::cylinderFinished()
{
  ui.mViewportX->setSelectedCylinder(&cylinders.back(), false);
  ui.mViewportY->setSelectedCylinder(&cylinders.back(), false);
  ui.mViewportZ->setSelectedCylinder(&cylinders.back(), false);
  ui.mViewportXYZ->setSelectedCylinder(&cylinders.back(), false);

  ui.lstCylinders->setCurrentRow(ui.lstCylinders->count() - 1);
  ui.lstCylinders->setEnabled(true);
  ui.btnNewCylinder->setEnabled(true);
  if (ui.lstCylinders->count() > 0) ui.btnRemoveCylinder->setEnabled(true);
}

void Mainframe::generateLabelButtons()
{
  const int BtnsPerRow = 5;

  std::map<uint32_t, std::string> label_names;
  std::map<uint32_t, ColorGL> label_colors;

  getLabelNames("labels.xml", label_names);
  getLabelColors("labels.xml", label_colors);

  labelButtonMapper = new QSignalMapper(this);

  uint32_t index = 0;

  QGridLayout* layout =
      dynamic_cast<QGridLayout*> (ui.labelsGroupBox->layout());
  if (layout == 0) throw "Layout of classesGroupBox is not a QGridLayout!";
  ui.labelsGroupBox->setLayout(layout);

  std::map<uint32_t, std::string>::const_iterator namesIter =
      label_names.begin();
  std::map<uint32_t, ColorGL>::const_iterator colorsIter = label_colors.begin();

  ui.cmbCylinderLabel->clear();

  for (; namesIter != label_names.end(); ++namesIter, ++colorsIter, ++index)
  {
    const int id = namesIter->first;
    const std::string& name = namesIter->second;
    const ColorGL& color = colorsIter->second;

    LabelButton* newButton = new LabelButton(ui.labelsGroupBox, color);
    newButton->setAutoFillBackground(true);
    labelButtons.push_back(newButton);
    labelIds[newButton] = id;
    layout->addWidget(newButton, std::floor((double) index / BtnsPerRow),
        index - std::floor((double) index / BtnsPerRow) * BtnsPerRow);

    newButton->setCheckable(true);
    newButton->setFixedSize(30, 30);

    newButton->setStatusTip(QString::fromStdString(name));

    /* connect the button with mapper which dispatches a signal with the index of the clicked button */
    labelButtonMapper->setMapping(newButton, newButton);
    connect(newButton, SIGNAL(released()), labelButtonMapper, SLOT(map()));

    QColor col(255.0f * color.R, 255.0f * color.G, 255.0f * color.B, 255);
    QPixmap pix(10, 10);
    pix.fill(col);
    QIcon icon(pix);
    QString text = QString::fromStdString(name);
    text += "(" + QString::number(id) + ")";
    ui.cmbCylinderLabel->insertItem(index, icon, text);
    idxLabelMap[index] = id;
    labelIdxMap[id] = index;
  }
  /** register only once the signal mapped to labelBtnReleased! **/
  connect(labelButtonMapper, SIGNAL(mapped(QWidget*)), this,
      SLOT(labelBtnReleased(QWidget*)));

  if (labelButtons.size() > 0) labelBtnReleased(labelButtons[0]);
}

void Mainframe::updateFiltering(bool value)
{
  if (value)
  {
    ui.mViewportX->setFilteredLabels(filteredLabels);
    ui.mViewportY->setFilteredLabels(filteredLabels);
    ui.mViewportZ->setFilteredLabels(filteredLabels);
    ui.mViewportXYZ->setFilteredLabels(filteredLabels);
  }
  else
  {
    std::vector<uint32_t> empty;
    ui.mViewportX->setFilteredLabels(empty);
    ui.mViewportY->setFilteredLabels(empty);
    ui.mViewportZ->setFilteredLabels(empty);
    ui.mViewportXYZ->setFilteredLabels(empty);
  }
}

void Mainframe::labelBtnReleased(QWidget* w)
{
  //  std::cout << "labelBtnReleased called." << std::endl;
  LabelButton* labelButton = dynamic_cast<LabelButton*> (w);
  if (labelButton == 0) return;
  if (labelIds.find(labelButton) == labelIds.end()) return;

  for (unsigned int i = 0; i < labelButtons.size(); ++i)
    labelButtons[i]->setChecked(false);

  labelButton->setChecked(true);
  uint32_t label_id = labelIds[labelButton];

  ui.mViewportX->setLabel(label_id);
  ui.mViewportY->setLabel(label_id);
  ui.mViewportZ->setLabel(label_id);
  ui.mViewportXYZ->setLabel(label_id);

  if (labelButton->isHighlighted())
  {
    filteredLabels.push_back(label_id);
    updateFiltering(ui.chkFilterLabels->isChecked());
  }
  else
  {
    std::vector<uint32_t> tempFilteredLabels;

    for (uint32_t i = 0; i < filteredLabels.size(); ++i)
      if (filteredLabels[i] != label_id) tempFilteredLabels.push_back(
          filteredLabels[i]);

    filteredLabels = tempFilteredLabels;
    updateFiltering(ui.chkFilterLabels->isChecked());
  }
}

void Mainframe::unsavedChanges()
{
  mChangesSinceLastSave = true;
}
