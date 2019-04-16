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

#include <QtGui/QClipboard>
#include <boost/lexical_cast.hpp>

using namespace glow;

Mainframe::Mainframe() : mChangesSinceLastSave(false) {
  ui.setupUi(this);

  connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(open()));
  connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(save()));

  /** initialize the paint button mapping **/
  connect(ui.btnBrushMode, &QToolButton::released,
          [this]() { changeMode(Viewport::PAINT, ui.btnBrushMode->isChecked()); });
  connect(ui.btnPolygonMode, &QToolButton::released,
          [this]() { changeMode(Viewport::POLYGON, ui.btnPolygonMode->isChecked()); });
  connect(ui.actionPaintMode, &QAction::triggered,
          [this]() { changeMode(Viewport::PAINT, ui.actionPaintMode->isChecked()); });
  connect(ui.actionPolygonMode, &QAction::triggered,
          [this]() { changeMode(Viewport::POLYGON, ui.actionPolygonMode->isChecked()); });

  ui.btnOverwrite->setDefaultAction(ui.actionOverwrite);
  ui.btnFilter->setDefaultAction(ui.actionFilter);

  connect(ui.mViewportXYZ, SIGNAL(labelingChanged()), this, SLOT(unsavedChanges()));

  //  connect(ui.btnOverwrite, &QToolButton::released, [this]() {
  //    ui.mViewportXYZ->setOverwrite(ui.btnOverwrite->isChecked());
  //    ui.actionOverwrite->setChecked(ui.btnOverwrite->isChecked());
  //  });
  connect(ui.actionOverwrite, &QAction::triggered, [this]() {
    ui.mViewportXYZ->setOverwrite(ui.actionOverwrite->isChecked());
    ui.btnOverwrite->setChecked(ui.actionOverwrite->isChecked());
    lblOverwrite_.setEnabled(ui.actionOverwrite->isChecked());
  });

  connect(ui.spinPointSize, SIGNAL(valueChanged(int)), ui.mViewportXYZ, SLOT(setPointSize(int)));

  connect(ui.btnRadius5, &QToolButton::released, [this]() { changeRadius(10); });
  connect(ui.btnRadius10, &QToolButton::released, [this]() { changeRadius(25); });
  connect(ui.btnRadius20, &QToolButton::released, [this]() { changeRadius(50); });

  connect(ui.mRadiusSlider, SIGNAL(valueChanged(int)), this, SLOT(changeRadius(int)));
  connect(ui.sldTimeline, &QSlider::valueChanged, [this](int value) { setCurrentScanIdx(value); });
  connect(ui.btnForward, &QToolButton::released, [this]() { forward(); });

  connect(ui.btnBackward, &QToolButton::released, [this]() { backward(); });

  //  connect(ui.btnFilter, &QCheckBox::toggled, [this](bool value) { updateFiltering(value); });
  connect(ui.actionFilter, &QAction::toggled, [this](bool value) { updateFiltering(value); });

  connect(ui.chkShowRemission, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("remission", value); });

  connect(ui.chkRemoveGround, &QCheckBox::toggled, [this](bool value) { ui.mViewportXYZ->setGroundRemoval(value); });
  connect(ui.spinGroundThreshold, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [this](double value) { ui.mViewportXYZ->setGroundThreshold(value); });

  connect(ui.chkShowSingleScan, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("single scan", value); });

  connect(ui.wgtTileSelector, &TileSelectorWidget::tileSelected, [this](int32_t i, int32_t j) { setTileIndex(i, j); });

  connect(ui.actionCenterView, &QAction::triggered, [this]() { ui.mViewportXYZ->centerOnCurrentTile(); });
  connect(ui.actionShowImage, &QAction::triggered, [this]() {
    if (images_.size() > 0) {
      wImgWidget_->show();
      wImgWidget_->setImage(images_[ui.sldTimeline->value()]);
      ui.mViewportXYZ->setDrawingOption("show camera", true);
    }
  });

  connect(ui.actionReload, &QAction::triggered, [this]() {
    updateScans();
    mChangesSinceLastSave = false;
  });

  connect(ui.btnButtonLayoutA, &QToolButton::released, [this]() {
    ui.btnButtonLayoutA->setChecked(true);
    ui.btnButtonLayoutB->setChecked(false);
    updateLabelButtons();
  });

  connect(ui.btnButtonLayoutB, &QToolButton::released, [this]() {
    ui.btnButtonLayoutB->setChecked(true);
    ui.btnButtonLayoutA->setChecked(false);
    updateLabelButtons();
  });

  connect(ui.cmbRootCategory, &QComboBox::currentTextChanged, [this]() { updateLabelButtons(); });

  connect(this, &Mainframe::readerFinshed, this, &Mainframe::updateScans);
  connect(this, &Mainframe::readerStarted, this, &Mainframe::activateSpinner);

  connect(ui.rdoMoving, &QRadioButton::released, [this]() { updateMovingStatus(true); });
  connect(ui.rdoStatic, &QRadioButton::released, [this]() { updateMovingStatus(false); });

  connect(ui.actionAutomaticallySave, &QAction::toggled, [this](bool toggled) {
    if (!toggled)
      mSaveTimer_.stop();
    else
      mSaveTimer_.start(180000);
  });

  connect(&mSaveTimer_, SIGNAL(timeout()), this, SLOT(save()));

  connect(ui.chkShowScanRange, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("show scan range", value); });
  connect(ui.spinRangeBegin, static_cast<void (QSpinBox::*)(int32_t)>(&QSpinBox::valueChanged),
          [this](int32_t value) { ui.mViewportXYZ->setScanRange(value, ui.spinRangeEnd->value()); });
  connect(ui.spinRangeEnd, static_cast<void (QSpinBox::*)(int32_t)>(&QSpinBox::valueChanged),
          [this](int32_t value) { ui.mViewportXYZ->setScanRange(ui.spinRangeBegin->value(), value); });

  // ------------------------------------------
  // Removal with plane in arbitrary normal direction
  // ------------------------------------------

  // Checkbox for removal in arbitrary normal direction
  connect(ui.chkPlaneRemovalNormal, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setPlaneRemovalNormal(value); });

  connect(ui.sldPlaneThresholdNormal, &QSlider::valueChanged, [this]() {
    ui.mViewportXYZ->setPlaneRemovalNormalParams(
        ui.sldPlaneThresholdNormal->value() / 100.0f, ui.sldPlaneNormalA1->value() / 8.0f,
        ui.sldPlaneNormalA2->value() / 4.0f, 0, ui.rdoPlaneBelowNormal->isChecked() ? -1.0f : 1.0f);
  });

  // Sliders to select normal parameters
  connect(ui.sldPlaneNormalA1, &QSlider::valueChanged, [this]() {
    ui.mViewportXYZ->setPlaneRemovalNormalParams(
        ui.sldPlaneThresholdNormal->value() / 100.0f, ui.sldPlaneNormalA1->value() / 8.0f,
        ui.sldPlaneNormalA2->value() / 4.0f, 0, ui.rdoPlaneBelowNormal->isChecked() ? -1.0f : 1.0f);
  });

  connect(ui.sldPlaneNormalA2, &QSlider::valueChanged, [this]() {
    ui.mViewportXYZ->setPlaneRemovalNormalParams(
        ui.sldPlaneThresholdNormal->value() / 100.0f, ui.sldPlaneNormalA1->value() / 8.0f,
        ui.sldPlaneNormalA2->value() / 4.0f, 0, ui.rdoPlaneBelowNormal->isChecked() ? -1.0f : 1.0f);
  });

  // Radio buttons to select orientation
  connect(ui.rdoPlaneAboveNormal, &QRadioButton::released, [this]() {
    ui.rdoPlaneBelowNormal->setChecked(false);
    ui.mViewportXYZ->setPlaneRemovalNormalParams(ui.sldPlaneThresholdNormal->value() / 100.0f,
                                                 ui.sldPlaneNormalA1->value() / 8.0f,
                                                 ui.sldPlaneNormalA2->value() / 8.0f, 0, 1.0f);
  });

  connect(ui.rdoPlaneBelowNormal, &QRadioButton::released, [this]() {
    ui.rdoPlaneAboveNormal->setChecked(false);
    ui.mViewportXYZ->setPlaneRemovalNormalParams(ui.sldPlaneThresholdNormal->value() / 100.0f,
                                                 ui.sldPlaneNormalA1->value() / 8.0f,
                                                 ui.sldPlaneNormalA2->value() / 8.0f, 0, -1.0f);
  });

  connect(ui.chkCarPoseAsBase, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("carAsBase", value); });

  connect(ui.chkShowPlane, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("show plane", value); });

  // ------------------------------------------
  // Camera Projection
  // ------------------------------------------

  /** load cameras**/
  std::vector<std::string> names = ui.mViewportXYZ->getCameraNames();

  for (auto name : names) {
    QAction* camact = new QAction(QString::fromStdString(name), this);
    camact->setCheckable(true);
    ui.menuCamera_Control->addAction(camact);
    connect(camact, &QAction::toggled, [this, name, camact](bool toggled) {
      if (toggled) {
        //        std::cout << cameras[name] << std::endl;
        ui.mViewportXYZ->setCameraByName(name);
        foreach (QAction* action, ui.menuCamera_Control->actions()) {
          if (action == camact) continue;
          action->setChecked(false);
        }
        // camact->setChecked(true);
      }
    });
  }

  connect(ui.actionPerspectiveProjection, &QAction::triggered, [this]() {
    ui.actionPerspectiveProjection->setChecked(true);
    ui.actionOrthographic->setChecked(false);
    ui.mViewportXYZ->setCameraProjection(Viewport::CameraProjection::perspective);
  });

  connect(ui.actionOrthographic, &QAction::triggered, [this]() {
    ui.actionPerspectiveProjection->setChecked(false);
    ui.actionOrthographic->setChecked(true);
    ui.mViewportXYZ->setCameraProjection(Viewport::CameraProjection::orthographic);
  });

  connect(ui.btnInitalizeInstances, &QPushButton::released, [this]() { ui.mViewportXYZ->initializeInstanceLables(); });
  connect(ui.chkDrawInstanceMap, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("draw instancemap", value); });
  connect(ui.chkDrawInstances, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("draw instances", value); });

  connect(ui.sldGamma, &QSlider::valueChanged, [this](int32_t value) {
    float gamma = float(value) / 10.0f;
    ui.lblGammaValue->setText(QString::number(gamma));
    ui.mViewportXYZ->setGammaCorrection(gamma);
  });

  connect(ui.cmbColormap, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          [this](int32_t idx) { ui.mViewportXYZ->setRemissionColorMap(idx); });

  /** load labels and colors **/
  std::map<uint32_t, glow::GlColor> label_colors;

  getLabelNames("labels.xml", label_names);
  getLabelColors("labels.xml", label_colors);

  ui.mViewportXYZ->setLabelColors(label_colors);
  ui.mViewportXYZ->setGroundThreshold(ui.spinGroundThreshold->value());

  generateLabelButtons();

  readConfig();

  initializeIcons();

  wImgWidget_ = new ImageViewer(nullptr, Qt::Window | Qt::WindowStaysOnTopHint);
  wImgWidget_->resize(1241, 376);
  ui.mViewportXYZ->update();

  lblNumPoints_.setText("0 ");
  lblNumPoints_.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  lblNumPoints_.setMinimumWidth(100);
  progressLabeled_.setMaximum(100);
  progressLabeled_.setMinimum(0);
  progressLabeled_.setTextVisible(true);
  progressLabeled_.setMinimumWidth(75);
  progressLabeled_.setMaximumWidth(75);
  lblOverwrite_.setText(" OVERWRITE ");

  ui.statusbar->addPermanentWidget(&lblOverwrite_);
  ui.statusbar->addPermanentWidget(&lblNumPoints_);
  ui.statusbar->addPermanentWidget(&progressLabeled_);

  info_ = new QWidget(this, Qt::FramelessWindowHint);
  info_->setAutoFillBackground(true);
  info_->setLayout(new QHBoxLayout);
  QLabel* label = new QLabel("Please wait while writing labels to disk.");
  label->setAlignment(Qt::AlignCenter);
  info_->layout()->addWidget(label);
  info_->hide();

  connect(ui.actionScreenshot, &QAction::triggered, [this]() {
    QImage img = ui.mViewportXYZ->grabFrameBuffer();
    img.save("screenshot.png");
    QApplication::clipboard()->setImage(img);
  });
  //
    ui.mViewportXYZ->resize(1000, 800);
    ui.mViewportXYZ->update();
}

Mainframe::~Mainframe() {}

void Mainframe::closeEvent(QCloseEvent* event) {
  if (mChangesSinceLastSave) {
    int32_t ret =
        QMessageBox::warning(this, tr("Unsaved changes."), tr("The annotation has been modified.\n"
                                                              "Do you want to save your changes?"),
                             QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Save);
    if (ret == QMessageBox::Save) {
      save();
    } else if (ret == QMessageBox::Cancel) {
      event->ignore();
      return;
    }
  }

  event->accept();

  wImgWidget_->close();
}

void Mainframe::open() {
  if (readerFuture_.valid()) readerFuture_.wait();

  if (mChangesSinceLastSave) {
    int32_t ret =
        QMessageBox::warning(this, tr("Unsaved changes."), tr("The annotation has been modified.\n"
                                                              "Do you want to save your changes?"),
                             QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Save);
    if (ret == QMessageBox::Save) {
      save();
    } else if (ret == QMessageBox::Cancel) {
      return;
    }
  }

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
    const auto& tile = reader_.getTile(Eigen::Vector3f::Zero());
    readerFuture_ = std::async(std::launch::async, &Mainframe::readAsync, this, tile.i, tile.j);
    ui.wgtTileSelector->initialize(reader_.getTiles(), reader_.numTiles().x(), reader_.numTiles().y());

    ui.wgtTileSelector->setTrajectory(reader_.getTileTrajectory());
    ui.wgtTileSelector->setSelected(tile.i, tile.j);

    lastDirectory = base_dir.absolutePath();

    changeMode(Viewport::NONE, true);

    QString title = "Point Labeler - ";
    title += QFileInfo(retValue).completeBaseName();
    setWindowTitle(title);

    mChangesSinceLastSave = false;
  }
}

void Mainframe::save() {
  int32_t w = 300, h = 150;
  info_->setGeometry(x() + width() / 2 - 0.5 * w, y() + height() / 2 - 0.5 * h, w, h);
  info_->show();

  statusBar()->showMessage("Writing labels...");
  ui.mViewportXYZ->updateLabels();
  reader_.update(indexes_, labels_);

  progressLabeled_.setValue(100.0f * ui.mViewportXYZ->labeledPointCount() / ui.mViewportXYZ->loadedPointCount());

  mChangesSinceLastSave = false;
  statusBar()->clearMessage();
  info_->close();
}

void Mainframe::changeRadius(int value) {
  ui.btnRadius5->setChecked(false);
  ui.btnRadius10->setChecked(false);
  ui.btnRadius20->setChecked(false);

  switch (value) {
    case 10:
      ui.btnRadius5->setChecked(true);
      break;
    case 25:
      ui.btnRadius10->setChecked(true);
      break;
    case 50:
      ui.btnRadius20->setChecked(true);
      break;
  }

  ui.mRadiusSlider->setValue(value);

  ui.mViewportXYZ->setRadius(value);
}

void Mainframe::changeMode(int mode, bool checked) {
  //  std::cout << "called changedMode(" << ((mode == Viewport::PAINT) ? "PAINT" : "POLYGON") << ", "
  //            << (checked ? "true" : "false") << std::endl;

  // TODO find better way.
  if (!checked) {
    ui.mViewportXYZ->setMode(Viewport::NONE);

    if (mode == Viewport::PAINT) {
      ui.btnBrushMode->setChecked(false);
      ui.actionPaintMode->setChecked(false);
    }

    if (mode == Viewport::POLYGON) {
      ui.btnPolygonMode->setChecked(false);
      ui.actionPolygonMode->setChecked(false);
    }
  }

  if (checked) {
    if (mode == Viewport::PAINT) {
      //      std::cout << "triggered paint mode." << std::endl;
      ui.mViewportXYZ->setMode(Viewport::PAINT);

      ui.btnPolygonMode->setChecked(false);
      ui.actionPolygonMode->setChecked(false);

      ui.btnBrushMode->setChecked(true);
      ui.actionPaintMode->setChecked(true);

      //    ui.mTools->setCurrentIndex(1);
    }

    if (mode == Viewport::POLYGON) {
      //      std::cout << "triggered polygon mode." << std::endl;
      ui.mViewportXYZ->setMode(Viewport::POLYGON);

      ui.btnBrushMode->setChecked(false);
      ui.actionPaintMode->setChecked(false);

      ui.btnPolygonMode->setChecked(true);
      ui.actionPolygonMode->setChecked(true);

      //    ui.mTools->setCurrentIndex(1);
    }
  }
}

void Mainframe::generateLabelButtons() {
  const int BtnsPerRow = 5;

  std::map<uint32_t, GlColor> label_colors;

  getLabels("labels.xml", labelDefinitions_);

  labelButtonMapper = new QSignalMapper(this);

  uint32_t index = 0;
  for (uint32_t i = 0; i < labelDefinitions_.size(); ++i) {
    //    const int32_t id = labelDefinitions_[i].id;
    const std::string name = labelDefinitions_[i].name;
    const GlColor color = labelDefinitions_[i].color;

    LabelButton* newButton =
        new LabelButton(this, QString::fromStdString(name), QColor(color.R * 255, color.G * 255, color.B * 255));
    newButton->setAutoFillBackground(true);
    labelButtons.push_back(newButton);
    labelButtonIdx_[newButton] = i;
    ui.labelsGroupBox->addWidget(newButton, std::floor((double)index / BtnsPerRow),
                                 index - std::floor((double)index / BtnsPerRow) * BtnsPerRow);

    catButtons_["all"].push_back(newButton);
    catButtons_[labelDefinitions_[i].rootCategory].push_back(newButton);

    newButton->setStatusTip(QString::fromStdString(name));
    newButton->setToolTip(QString::fromStdString(name));

    /* connect the button with mapper which dispatches a signal with the index of the clicked button */
    labelButtonMapper->setMapping(newButton, newButton);
    connect(newButton, SIGNAL(released()), labelButtonMapper, SLOT(map()));
    ++index;
  }

  for (auto it = catButtons_.begin(); it != catButtons_.end(); ++it) {
    if (it->first == "all") continue;
    ui.cmbRootCategory->addItem(QString::fromStdString(it->first));
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
  if (labelButton == nullptr) return;

  for (uint32_t i = 0; i < labelButtons.size(); ++i) labelButtons[i]->setChecked(false);

  labelButton->setChecked(true);
  selectedLabelButtonIdx_ = labelButtonIdx_[labelButton];

  uint32_t label_id = labelDefinitions_[selectedLabelButtonIdx_].id;
  bool potentiallyMoving = labelDefinitions_[selectedLabelButtonIdx_].potentiallyMoving;

  ui.rdoMoving->setEnabled(potentiallyMoving);
  ui.rdoStatic->setEnabled(potentiallyMoving);

  if (potentiallyMoving && ui.rdoMoving->isChecked()) {
    label_id = labelDefinitions_[selectedLabelButtonIdx_].id_moving;
  }

  ui.mViewportXYZ->setLabel(label_id);

  if (labelButton->isHighlighted()) {
    if (!contains(filteredLabels, label_id)) {
      filteredLabels.push_back(label_id);
      updateFiltering(ui.btnFilter->isChecked());
    }
  } else {
    std::vector<uint32_t> tempFilteredLabels;

    for (uint32_t i = 0; i < filteredLabels.size(); ++i)
      if (filteredLabels[i] != label_id) tempFilteredLabels.push_back(filteredLabels[i]);

    filteredLabels = tempFilteredLabels;
    updateFiltering(ui.btnFilter->isChecked());
  }

  uint32_t maxRecently = 10;
  std::vector<LabelButton*> newButtons;
  newButtons.push_back(labelButton);
  for (auto btn : catButtons_["recently"]) {
    if (btn == labelButton) continue;
    newButtons.push_back(btn);
  }
  if (newButtons.size() > maxRecently) newButtons.resize(maxRecently);
  std::sort(newButtons.begin(), newButtons.end(), [this](LabelButton* a, LabelButton* b) {
    return labelDefinitions_[labelButtonIdx_[a]].id < labelDefinitions_[labelButtonIdx_[b]].id;
  });
  catButtons_["recently"] = newButtons;

  ui.txtSelectedLabel->setText(QString::fromStdString(label_names[label_id]));
}

void Mainframe::unsavedChanges() { mChangesSinceLastSave = true; }

void Mainframe::setTileIndex(uint32_t i, uint32_t j) {
  if (readerFuture_.valid()) readerFuture_.wait();

  if (mChangesSinceLastSave) {
    int32_t ret = QMessageBox::warning(this, tr("Unsaved changes."), tr("The annotation has been modified.\n"
                                                                        "Do you want to save your changes?"),
                                       QMessageBox::Save | QMessageBox::Discard, QMessageBox::Save);
    if (ret == QMessageBox::Save) save();
  }

  readerFuture_ = std::async(std::launch::async, &Mainframe::readAsync, this, i, j);
}

void Mainframe::setCurrentScanIdx(int32_t idx) {
  ui.mViewportXYZ->setDrawingOption("show camera", wImgWidget_->isVisible());
  ui.mViewportXYZ->setScanIndex(idx);
  if (images_.size() > uint32_t(idx)) wImgWidget_->setImage(images_[idx]);
}

void Mainframe::readAsync(uint32_t i, uint32_t j) {
  // TODO progress indicator.
  emit readerStarted();

  std::vector<uint32_t> indexes;
  std::vector<PointcloudPtr> points;
  std::vector<LabelsPtr> labels;
  std::vector<std::string> images;

  //  std::vector<uint32_t> oldIndexes = indexes_;
  //  std::vector<LabelsPtr> oldLabels = labels_;

  reader_.retrieve(i, j, indexes, points, labels, images);

  indexes_ = indexes;
  points_ = points;
  labels_ = labels;
  images_ = images;

  //  // find difference.
  //  std::vector<uint32_t> diff_indexes;
  //  index_difference(oldLabels, labels_, diff_indexes);
  //
  //  std::vector<uint32_t> removedIndexes;
  //  std::vector<LabelsPtr> removedLabels;
  //
  //  for (auto index : diff_indexes) {
  //    removedIndexes.push_back(oldIndexes[index]);
  //    removedLabels.push_back(oldLabels[index]);
  //  }
  //  // only update really needed label files.
  //  //  reader_.update(removedIndexes, removedLabels);

  const auto& tile = reader_.getTile(i, j);
  ui.mViewportXYZ->setTileInfo(tile.x, tile.y, tile.size);

  emit readerFinshed();
}

void Mainframe::activateSpinner() {
  spinner = new WaitingSpinnerWidget(statusBar(), false, false);
  //  statusBar()->addPermanentWidget(spinner);

  spinner->setInnerRadius(7);
  spinner->setLineLength(3);
  spinner->start();
  statusBar()->showMessage("     Reading scans...");
  ui.wgtTileSelector->setEnabled(false);
}

void Mainframe::updateScans() {
  if (spinner != nullptr) {
    spinner->stop();
    statusBar()->removeWidget(spinner);
    delete spinner;
    spinner = nullptr;
  }

  statusBar()->clearMessage();

  ui.mViewportXYZ->setPoints(points_, labels_);
  ui.sldTimeline->setMaximum(indexes_.size() - 1);
  ui.sldTimeline->setValue(0);
  ui.wgtTileSelector->setEnabled(true);
  mChangesSinceLastSave = false;
  QString number = QString::number(ui.mViewportXYZ->loadedPointCount());
  QString dotted_number;
  while (number.size() > 3) {
    dotted_number = QString(".") + number.right(3) + dotted_number;
    number.chop(3);
  }
  dotted_number = number + dotted_number;
  lblNumPoints_.setText(dotted_number + " ");
  progressLabeled_.setValue(100.0f * ui.mViewportXYZ->labeledPointCount() / ui.mViewportXYZ->loadedPointCount());

  ui.spinRangeBegin->setValue(0);
  ui.spinRangeBegin->setMaximum(indexes_.size() - 1);
  ui.spinRangeEnd->setValue(indexes_.size() - 1);
  ui.spinRangeEnd->setMaximum(indexes_.size() - 1);
}

void Mainframe::forward() {
  int32_t value = ui.sldTimeline->value() + 1;
  if (value < int32_t(reader_.count())) ui.sldTimeline->setValue(value);
  ui.btnBackward->setEnabled(true);
  if (value == int32_t(reader_.count()) - 1) ui.btnForward->setEnabled(false);
}

void Mainframe::backward() {
  int32_t value = ui.sldTimeline->value() - 1;
  if (value >= 0) ui.sldTimeline->setValue(value);
  ui.btnForward->setEnabled(true);
  if (value == 0) ui.btnBackward->setEnabled(false);
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
    if (tokens[0] == "flip mouse buttons") {
      float value = boost::lexical_cast<float>(trim(tokens[1]));
      ui.mViewportXYZ->setFlipMouseButtons((value == 0) ? false : true);
      std::cout << "-- Setting 'flip mouse buttons' to " << ((value == 0) ? "false" : "true") << std::endl;
    }
    if (tokens[0] == "camera") {
      std::string value = trim(tokens[1]);
      auto cameras = ui.mViewportXYZ->getCameraNames();
      bool found = false;
      for (auto name : cameras) {
        if (name == value) ui.mViewportXYZ->setCameraByName(name);
      }
      if (found) {
        std::cout << "-- Setting 'camera' to " << value << std::endl;
      } else {
        std::cout << "-- [ERROR] Could not set 'camera' to " << value << ". Undefined camera. Using default."
                  << std::endl;
      }
    }
  }

  in.close();
}

void Mainframe::initializeIcons() {
  std::string assertDir = QDir::currentPath().toStdString() + "/../assets/";
  std::cout << QDir::currentPath().toStdString() << std::endl;
  {
    QIcon icon;
    icon.addPixmap(QPixmap(QString::fromStdString(assertDir + "brush.png")));

    ui.actionPaintMode->setIcon(icon);
    ui.btnBrushMode->setIcon(icon);
  }

  {
    QIcon icon;
    icon.addPixmap(QPixmap(QString::fromStdString(assertDir + "polygon.png")));
    ui.actionPolygonMode->setIcon(icon);
    ui.btnPolygonMode->setIcon(icon);
  }

  {
    QIcon icon;
    icon.addPixmap(QPixmap(QString::fromStdString(assertDir + "filter.png")));
    ui.actionFilter->setIcon(icon);
  }

  {
    QIcon icon;
    icon.addPixmap(QPixmap(QString::fromStdString(assertDir + "overwrite_on.png")), QIcon::Normal, QIcon::On);
    icon.addPixmap(QPixmap(QString::fromStdString(assertDir + "overwrite_off.png")), QIcon::Normal, QIcon::Off);
    ui.actionOverwrite->setIcon(icon);
  }

  {
    QIcon icon;
    icon.addPixmap(QPixmap(QString::fromStdString(assertDir + "open.png")));

    ui.actionOpen->setIcon(icon);
  }

  {
    QIcon icon;
    icon.addPixmap(QPixmap(QString::fromStdString(assertDir + "save.png")));

    ui.actionSave->setIcon(icon);
  }

  {
    QIcon icon;
    icon.addPixmap(QPixmap(QString::fromStdString(assertDir + "reload.png")));

    ui.actionReload->setIcon(icon);
  }

  {
    QIcon icon;
    icon.addPixmap(QPixmap(QString::fromStdString(assertDir + "centerview.png")));

    ui.actionCenterView->setIcon(icon);
  }

  {
    QIcon icon;
    icon.addPixmap(QPixmap(QString::fromStdString(assertDir + "image.png")));

    ui.actionShowImage->setIcon(icon);
  }

  {
    QIcon icon;
    icon.addPixmap(QPixmap(QString::fromStdString(assertDir + "layoutA.png")));

    ui.btnButtonLayoutA->setIcon(icon);
  }

  {
    QIcon icon;
    icon.addPixmap(QPixmap(QString::fromStdString(assertDir + "layoutB.png")));

    ui.btnButtonLayoutB->setIcon(icon);
  }
}

void Mainframe::keyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_Right:
      if (ui.btnForward->isEnabled()) forward();
      return;

    case Qt::Key_Left:
      if (ui.btnBackward->isEnabled()) backward();
      return;

    case Qt::Key_O:
      ui.actionOverwrite->trigger();
      return;

    case Qt::Key_F:
      ui.actionFilter->trigger();
      return;

    case Qt::Key_Plus:
      ui.spinPointSize->setValue(std::min<int32_t>(ui.spinPointSize->value() + 1, 10));
      return;

    case Qt::Key_Minus:
      ui.spinPointSize->setValue(std::max<int32_t>(ui.spinPointSize->value() - 1, 1));
      return;

    case Qt::Key_1:
      changeMode(Viewport::PAINT, true);
      return;

    case Qt::Key_2:
      changeMode(Viewport::POLYGON, true);
      return;
    case Qt::Key_R:
      ui.chkRemoveGround->toggle();
      return;
    default:
      if (!ui.mViewportXYZ->hasFocus()) ui.mViewportXYZ->keyPressEvent(event);
      return;
  }
}

void Mainframe::keyReleaseEvent(QKeyEvent* event) {
  double value = ui.spinGroundThreshold->value();
  double step = ui.spinGroundThreshold->singleStep();

  switch (event->key()) {
    case Qt::Key_Right:
    case Qt::Key_Left:
    case Qt::Key_O:
    case Qt::Key_F:
    case Qt::Key_Plus:
    case Qt::Key_Minus:
    case Qt::Key_1:
    case Qt::Key_2:
      return;
    case Qt::Key_F1:
      changeRadius(10);
      return;
    case Qt::Key_F2:
      changeRadius(25);
      return;
    case Qt::Key_F3:
      changeRadius(50);
      return;
    case Qt::Key_E:
      ui.spinGroundThreshold->setValue(value + step);
      return;
    case Qt::Key_Q:
      ui.spinGroundThreshold->setValue(value - step);
      return;
    default:
      if (!ui.mViewportXYZ->hasFocus()) ui.mViewportXYZ->keyReleaseEvent(event);
      return;
  }
}

void Mainframe::updateMovingStatus(bool isMoving) {
  if (selectedLabelButtonIdx_ < 0) return;
  if (!labelDefinitions_[selectedLabelButtonIdx_].potentiallyMoving) return;

  ui.rdoStatic->setChecked(!isMoving);
  ui.rdoMoving->setChecked(isMoving);

  // a little bit hacky, but here we also ensure that the highlighting is correct.
  for (uint32_t i = 0; i < labelDefinitions_.size(); ++i) {
    if (!labelDefinitions_[i].potentiallyMoving) continue;
    uint32_t id = (isMoving) ? labelDefinitions_[i].id_moving : labelDefinitions_[i].id;

    labelButtons[i]->setHighlighted(contains(filteredLabels, id));
  }

  // update the label.

  uint32_t label_id = labelDefinitions_[selectedLabelButtonIdx_].id;
  bool potentiallyMoving = labelDefinitions_[selectedLabelButtonIdx_].potentiallyMoving;

  if (potentiallyMoving && ui.rdoMoving->isChecked()) {
    label_id = labelDefinitions_[selectedLabelButtonIdx_].id_moving;
  }

  ui.mViewportXYZ->setLabel(label_id);

  ui.txtSelectedLabel->setText(QString::fromStdString(label_names[label_id]));
}

void Mainframe::updateLabelButtons() {
  for (auto w : labelButtons) {
    //    ui.labelsGroupBox->removeWidget(w);
    w->setVisible(false);  // ensures that the button is removed.
  }

  uint32_t btnsPerRow = 5;
  if (ui.btnButtonLayoutB->isChecked()) btnsPerRow = 2;

  std::vector<LabelButton*>* btns = &(catButtons_["all"]);
  std::string category = ui.cmbRootCategory->currentText().toStdString();
  btns = &(catButtons_)[category];

  for (uint32_t index = 0; index < btns->size(); ++index) {
    auto btn = (*btns)[index];
    btn->setVisible(true);
    ui.labelsGroupBox->addWidget(btn, std::floor((double)index / btnsPerRow),
                                 index - std::floor((double)index / btnsPerRow) * btnsPerRow);
  }
}
