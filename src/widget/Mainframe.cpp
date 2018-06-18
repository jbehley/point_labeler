#include "Mainframe.h"

#include <fstream>
#include <iostream>
#include <map>

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtWidgets/QFileDialog>

#include "../data/label_utils.h"
#include "../data/misc.h"

#include <QtWidgets/QMessageBox>

#include <boost/lexical_cast.hpp>

using namespace glow;

Mainframe::Mainframe() : mChangesSinceLastSave(false) {
  ui.setupUi(this);

  connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(open()));
  connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(save()));

  /** initialize the paint button mapping **/
  connect(ui.btnBrushMode, &QToolButton::released, [this]() { changeMode(Viewport::PAINT); });
  connect(ui.btnPolygonMode, &QToolButton::toggled, [this]() { changeMode(Viewport::POLYGON); });

  connect(ui.mViewportXYZ, SIGNAL(labelingChanged()), this, SLOT(unsavedChanges()));

  connect(ui.btnOverwrite, SIGNAL(toggled(bool)), ui.mViewportXYZ, SLOT(setOverwrite(bool)));

  connect(ui.spinPointSize, SIGNAL(valueChanged(int)), ui.mViewportXYZ, SLOT(setPointSize(int)));

  connect(ui.btnRadius5, &QToolButton::released, [this]() { changeRadius(5); });
  connect(ui.btnRadius10, &QToolButton::released, [this]() { changeRadius(10); });
  connect(ui.btnRadius20, &QToolButton::released, [this]() { changeRadius(20); });

  connect(ui.mRadiusSlider, SIGNAL(valueChanged(int)), this, SLOT(changeRadius(int)));
  connect(ui.sldTimeline, &QSlider::valueChanged, [this](int value) { setCurrentScanIdx(value); });
  connect(ui.btnForward, &QToolButton::released, [this]() {
    int32_t value = ui.sldTimeline->value() + 1;
    if (value < int32_t(reader_.count())) ui.sldTimeline->setValue(value);
    ui.btnBackward->setEnabled(true);
    if (value == int32_t(reader_.count()) - 1) ui.btnForward->setEnabled(false);
  });

  connect(ui.btnBackward, &QToolButton::released, [this]() {
    int32_t value = ui.sldTimeline->value() - 1;
    if (value >= 0) ui.sldTimeline->setValue(value);
    ui.btnForward->setEnabled(true);
    if (value == 0) ui.btnBackward->setEnabled(false);
  });

  connect(ui.chkFilterLabels, &QCheckBox::toggled, [this](bool value) { updateFiltering(value); });

  connect(ui.chkShowRemission, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("remission", value); });

  connect(ui.chkShowColor, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("color", value); });

  connect(ui.chkRemoveGround, &QCheckBox::toggled, [this](bool value) { ui.mViewportXYZ->setGroundRemoval(value); });
  connect(ui.spinGroundThreshold, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [this](double value) { ui.mViewportXYZ->setGroundThreshold(value); });

  connect(ui.chkShowSingleScan, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("single scan", value); });

  /** load labels and colors **/
  std::map<uint32_t, std::string> label_names;
  std::map<uint32_t, glow::GlColor> label_colors;

  getLabelNames("labels.xml", label_names);
  getLabelColors("labels.xml", label_colors);

  ui.mViewportXYZ->setLabelColors(label_colors);

  generateLabelButtons();

  readConfig();
}

Mainframe::~Mainframe() {}

void Mainframe::closeEvent(QCloseEvent* event) {
  //  if (mChangesSinceLastSave) {
  //    int ret = QMessageBox::warning(this, tr("Unsaved changes."), tr("The annotation has been modified.\n"
  //                                                                    "Do you want to save your changes?"),
  //                                   QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
  //                                   QMessageBox::Save);
  //    if (ret == QMessageBox::Save)
  //      save();
  //    else if (ret == QMessageBox::Cancel) {
  //      event->ignore();
  //      return;
  //    }
  //  }
  ui.mViewportXYZ->updateLabels();
  reader_.update(indexes_, labels_);

  event->accept();
}

void Mainframe::open() {
  //  if (mChangesSinceLastSave) {
  //    int ret = QMessageBox::warning(this, tr("Unsaved changes."), tr("The annotation has been modified.\n"
  //                                                                    "Do you want to save your changes?"),
  //                                   QMessageBox::Save | QMessageBox::Cancel | QMessageBox::Discard,
  //                                   QMessageBox::Save);
  //    if (ret == QMessageBox::Save)
  //      save();
  //    else if (ret == QMessageBox::Cancel)
  //      return;
  //  }

  QString retValue =
      QFileDialog::getExistingDirectory(this, "Select scan directory", lastDirectory, QFileDialog::ShowDirsOnly);

  if (!retValue.isNull()) {
    QDir base_dir(retValue);

    if (!base_dir.exists("velodyne") || !base_dir.exists("poses.txt")) {
      std::cout << "[ERROR] velodyne or poses.txt missing." << std::endl;
      return;
    }

    reader_.initialize(retValue);

    //    ui.sldTimeline->setMaximum(reader_.count());
    ui.btnBackward->setEnabled(false);
    ui.btnForward->setEnabled(false);
    if (reader_.count() > 0) ui.btnForward->setEnabled(true);

    //    if (ui.sldTimeline->value() == 0) setCurrentScanIdx(0);
    //    ui.sldTimeline->setValue(0);
    reader_.retrieve(Eigen::Vector3f::Zero(), indexes_, points_, labels_);
    ui.mViewportXYZ->setPoints(points_, labels_);
    ui.sldTimeline->setMaximum(indexes_.size());
    ui.sldTimeline->setValue(0);
    const auto& tile = reader_.getTile(Eigen::Vector3f::Zero());
    ui.mViewportXYZ->setTileInfo(tile.x, tile.y, tile.size);

    lastDirectory = base_dir.absolutePath();

    changeMode(Viewport::NONE);

    QString title = "Point Labeler - ";
    title += QFileInfo(retValue).completeBaseName();
    setWindowTitle(title);

    mChangesSinceLastSave = false;
  }
}

void Mainframe::save() {
  ui.mViewportXYZ->updateLabels();
  reader_.update(indexes_, labels_);

  mChangesSinceLastSave = false;
}

void Mainframe::changeRadius(int value) {
  ui.btnRadius5->setChecked(false);
  ui.btnRadius10->setChecked(false);
  ui.btnRadius20->setChecked(false);

  switch (value) {
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

  ui.mViewportXYZ->setRadius(value);
}

void Mainframe::changeMode(int mode) {
  ui.mViewportXYZ->setMode(Viewport::NONE);

  if (mode == Viewport::PAINT && ui.btnBrushMode->isChecked()) /** PAINTMODE **/
  {
    std::cout << "triggered paint mode." << std::endl;
    ui.mViewportXYZ->setMode(Viewport::PAINT);

    ui.btnPolygonMode->setChecked(false);

    ui.mTools->setCurrentIndex(1);
  } else if (mode == Viewport::POLYGON && ui.btnPolygonMode->isChecked()) /** PAINTMODE **/
  {
    std::cout << "triggered polygon mode." << std::endl;
    ui.mViewportXYZ->setMode(Viewport::POLYGON);

    ui.btnBrushMode->setChecked(false);

    //    ui.mTools->setCurrentIndex(1);
  } else if (mode == Viewport::NONE) {
    ui.btnPolygonMode->setChecked(false);
    ui.btnBrushMode->setChecked(false);
  }
}

void Mainframe::generateLabelButtons() {
  const int BtnsPerRow = 5;

  std::map<uint32_t, std::string> label_names;
  std::map<uint32_t, GlColor> label_colors;

  getLabelNames("labels.xml", label_names);
  getLabelColors("labels.xml", label_colors);

  labelButtonMapper = new QSignalMapper(this);

  uint32_t index = 0;

  QGridLayout* layout = dynamic_cast<QGridLayout*>(ui.labelsGroupBox->layout());
  if (layout == 0) throw "Layout of classesGroupBox is not a QGridLayout!";
  ui.labelsGroupBox->setLayout(layout);

  std::map<uint32_t, std::string>::const_iterator namesIter = label_names.begin();
  std::map<uint32_t, GlColor>::const_iterator colorsIter = label_colors.begin();

  for (; namesIter != label_names.end(); ++namesIter, ++colorsIter, ++index) {
    const int id = namesIter->first;
    const std::string& name = namesIter->second;
    const GlColor& color = colorsIter->second;

    LabelButton* newButton = new LabelButton(ui.labelsGroupBox, QColor(color.R * 255, color.G * 255, color.B * 255));
    newButton->setAutoFillBackground(true);
    labelButtons.push_back(newButton);
    labelIds[newButton] = id;
    layout->addWidget(newButton, std::floor((double)index / BtnsPerRow),
                      index - std::floor((double)index / BtnsPerRow) * BtnsPerRow);

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

    idxLabelMap[index] = id;
    labelIdxMap[id] = index;
  }
  /** register only once the signal mapped to labelBtnReleased! **/
  connect(labelButtonMapper, SIGNAL(mapped(QWidget*)), this, SLOT(labelBtnReleased(QWidget*)));

  if (labelButtons.size() > 0) labelBtnReleased(labelButtons[0]);
}

void Mainframe::updateFiltering(bool value) {
  if (value) {
    ui.mViewportXYZ->setFilteredLabels(filteredLabels);
  } else {
    std::vector<uint32_t> empty;

    ui.mViewportXYZ->setFilteredLabels(empty);
  }
}

void Mainframe::labelBtnReleased(QWidget* w) {
  //  std::cout << "labelBtnReleased called." << std::endl;
  LabelButton* labelButton = dynamic_cast<LabelButton*>(w);
  if (labelButton == 0) return;
  if (labelIds.find(labelButton) == labelIds.end()) return;

  for (unsigned int i = 0; i < labelButtons.size(); ++i) labelButtons[i]->setChecked(false);

  labelButton->setChecked(true);
  uint32_t label_id = labelIds[labelButton];

  ui.mViewportXYZ->setLabel(label_id);

  if (labelButton->isHighlighted()) {
    filteredLabels.push_back(label_id);
    updateFiltering(ui.chkFilterLabels->isChecked());
  } else {
    std::vector<uint32_t> tempFilteredLabels;

    for (uint32_t i = 0; i < filteredLabels.size(); ++i)
      if (filteredLabels[i] != label_id) tempFilteredLabels.push_back(filteredLabels[i]);

    filteredLabels = tempFilteredLabels;
    updateFiltering(ui.chkFilterLabels->isChecked());
  }
}

void Mainframe::unsavedChanges() {
  mChangesSinceLastSave = true;
}

void Mainframe::setTileIndex(uint32_t i, uint32_t j) {
  std::vector<uint32_t> oldIndexes = indexes_;
  std::vector<LabelsPtr> oldLabels = labels_;

  reader_.retrieve(i, j, indexes_, points_, labels_);

  ui.mViewportXYZ->setPoints(points_, labels_);

  // find difference.
  std::vector<uint32_t> diff_indexes;
  index_difference(oldLabels, labels_, diff_indexes);

  std::vector<uint32_t> removedIndexes;
  std::vector<LabelsPtr> removedLabels;

  for (auto index : diff_indexes) {
    removedIndexes.push_back(oldIndexes[index]);
    removedLabels.push_back(oldLabels[index]);
  }
  // only update really needed label files.
  reader_.update(removedIndexes, removedLabels);
}

void Mainframe::setCurrentScanIdx(int32_t idx) {
  std::cout << "setting scan." << std::endl;
  ui.mViewportXYZ->setScanIndex(idx);
}

void Mainframe::readConfig() {
  std::ifstream in("settings.cfg");

  if (!in.is_open()) return;

  std::string line;
  in.peek();
  while (in.good() && !in.eof()) {
    std::getline(in, line);

    auto tokens = split(line, ":");
    if (tokens[0] == "max scans") {
      uint32_t numScans = boost::lexical_cast<uint32_t>(trim(tokens[1]));
      ui.mViewportXYZ->setMaximumScans(numScans);
      std::cout << "-- Setting 'max scans' to " << numScans << std::endl;
    }

    if (tokens[0] == "tile size") {
      float tileSize = boost::lexical_cast<float>(trim(tokens[1]));
      reader_.setTileSize(tileSize);
      std::cout << "-- Setting 'tile size' to " << tileSize << std::endl;
    }

    if (tokens[0] == "max range") {
      float range = boost::lexical_cast<float>(trim(tokens[1]));
      ui.mViewportXYZ->setMaxRange(range);
      reader_.setMaximumDistance(range);
      std::cout << "-- Setting 'max range' to " << range << std::endl;
    }

    if (tokens[0] == "min range") {
      float range = boost::lexical_cast<float>(trim(tokens[1]));
      ui.mViewportXYZ->setMinRange(range);
      std::cout << "-- Setting 'min range' to " << range << std::endl;
    }
  }

  in.close();
}
