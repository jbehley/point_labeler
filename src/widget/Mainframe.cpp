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

  connect(ui.chkShowColor, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("color", value); });

  connect(ui.chkRemoveGround, &QCheckBox::toggled, [this](bool value) { ui.mViewportXYZ->setGroundRemoval(value); });
  connect(ui.spinGroundThreshold, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [this](double value) { ui.mViewportXYZ->setGroundThreshold(value); });

  connect(ui.chkShowSingleScan, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("single scan", value); });

  connect(ui.wgtTileSelector, &TileSelectorWidget::tileSelected, [this](int32_t i, int32_t j) { setTileIndex(i, j); });
  connect(ui.chkShowAllPoints, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("show all points", value); });

  connect(this, &Mainframe::readerFinshed, this, &Mainframe::updateScans);
  connect(this, &Mainframe::readerStarted, this, &Mainframe::activateSpinner);

  /** load labels and colors **/
  std::map<uint32_t, std::string> label_names;
  std::map<uint32_t, glow::GlColor> label_colors;

  getLabelNames("labels.xml", label_names);
  getLabelColors("labels.xml", label_colors);

  ui.mViewportXYZ->setLabelColors(label_colors);
  ui.mViewportXYZ->setGroundThreshold(ui.spinGroundThreshold->value());

  generateLabelButtons();

  readConfig();

  initializeIcons();

  wImgWidget_ = new ImageViewer(nullptr, Qt::Window);
  wImgWidget_->resize(1241, 376);
  wImgWidget_->show();
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
  statusBar()->showMessage("Writing labels...");
  ui.mViewportXYZ->updateLabels();
  reader_.update(indexes_, labels_);

  event->accept();

  wImgWidget_->close();
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
    const auto& tile = reader_.getTile(Eigen::Vector3f::Zero());
    readerFuture_ = std::async(std::launch::async, &Mainframe::readAsync, this, tile.i, tile.j);
    ui.wgtTileSelector->initialize(reader_.getTiles(), reader_.numTiles().x(), reader_.numTiles().y());

    //    ui.mViewportXYZ->setTileInfo(tile.x, tile.y, tile.size);
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
  statusBar()->showMessage("Writing labels...");
  ui.mViewportXYZ->updateLabels();
  reader_.update(indexes_, labels_);

  mChangesSinceLastSave = false;
  statusBar()->clearMessage();
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
    newButton->setToolTip(QString::fromStdString(name));

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
    updateFiltering(ui.btnFilter->isChecked());
  } else {
    std::vector<uint32_t> tempFilteredLabels;

    for (uint32_t i = 0; i < filteredLabels.size(); ++i)
      if (filteredLabels[i] != label_id) tempFilteredLabels.push_back(filteredLabels[i]);

    filteredLabels = tempFilteredLabels;
    updateFiltering(ui.btnFilter->isChecked());
  }

  ui.txtSelectedLabel->setText(QString::fromStdString(label_names[label_id]));
}

void Mainframe::unsavedChanges() { mChangesSinceLastSave = true; }

void Mainframe::setTileIndex(uint32_t i, uint32_t j) {
  if (readerFuture_.valid()) readerFuture_.wait();
  readerFuture_ = std::async(std::launch::async, &Mainframe::readAsync, this, i, j);
}

void Mainframe::setCurrentScanIdx(int32_t idx) {
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

  std::vector<uint32_t> oldIndexes = indexes_;
  std::vector<LabelsPtr> oldLabels = labels_;

  reader_.retrieve(i, j, indexes, points, labels, images);

  indexes_ = indexes;
  points_ = points;
  labels_ = labels;
  images_ = images;

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
  glow::_CheckGlError(__FILE__, __LINE__);
  ui.mViewportXYZ->setPoints(points_, labels_);
  glow::_CheckGlError(__FILE__, __LINE__);

  ui.sldTimeline->setMaximum(indexes_.size());
  ui.sldTimeline->setValue(0);
  ui.wgtTileSelector->setEnabled(true);
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
}

void Mainframe::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_D || event->key() == Qt::Key_Right) {
    if (ui.btnForward->isEnabled()) forward();

  } else if (event->key() == Qt::Key_A || event->key() == Qt::Key_Left) {
    if (ui.btnBackward->isEnabled()) backward();
  }
}
