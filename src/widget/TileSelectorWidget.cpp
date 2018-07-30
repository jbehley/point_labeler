#include "TileSelectorWidget.h"
#include <cmath>

TileSelectorWidget::TileSelectorWidget(QWidget* parent, Qt::WindowFlags f) {}

void TileSelectorWidget::initialize(const std::vector<KittiReader::Tile>& tiles, uint32_t numTilesX,
                                    uint32_t numTilesY) {
  tiles_.clear();

  numTilesX_ = numTilesX;
  numTilesY_ = numTilesY;

  size_ = float(std::min(width(), height()) - 1) / (std::max(numTilesX_, numTilesY_) + 1);

  cx_ = width() - 0.5f * (width() - (numTilesY_ + 1) * size_);
  cy_ = height() - 0.5f * (height() - (numTilesX_ + 1) * size_);

  for (uint32_t i = 0; i < tiles.size(); ++i) {
    const auto& tile = tiles[i];
    if (tile.indexes.size() > 0) {
      uint32_t x = cx_ - tile.j * size_ - size_;
      uint32_t y = cy_ - tile.i * size_ - size_;
      tiles_.push_back(TileProxy(x, y, size_));
      tiles_.back().i = tile.i;
      tiles_.back().j = tile.j;
    }
  }

  selectedTile_ = -1;

  update();
}

void TileSelectorWidget::select(uint32_t i, uint32_t j) {
  setSelected(i, j);

  emit tileSelected(i, j);
}

void TileSelectorWidget::setSelected(uint32_t i, uint32_t j) {
  if (selectedTile_ > -1) {
    if (tiles_[selectedTile_].i == i && tiles_[selectedTile_].j == j) return;
  }

  for (uint32_t k = 0; k < tiles_.size(); ++k) {
    if (tiles_[k].i == i && tiles_[k].j == j) {
      if (selectedTile_ > -1) tiles_[selectedTile_].selected = false;
      selectedTile_ = k;
      tiles_[selectedTile_].selected = true;

      break;
    }
  }

  update();
}

void TileSelectorWidget::mouseMoveEvent(QMouseEvent* event) {}

void TileSelectorWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (size_ == 0) return;

  int32_t j = (cx_ - event->x()) / size_;
  int32_t i = (cy_ - event->y()) / size_;

  if (i < 0 || i >= int32_t(numTilesX_) || j < 0 || j >= int32_t(numTilesY_)) return;

  select(i, j);
}

void TileSelectorWidget::resizeEvent(QResizeEvent* event) {
  size_ = float(std::min(width(), height()) - 1) / (std::max(numTilesX_, numTilesY_) + 1);

  cx_ = width() - 0.5f * (width() - (numTilesY_ + 1) * size_);
  cy_ = height() - 0.5f * (height() - (numTilesX_ + 1) * size_);

  for (uint32_t i = 0; i < tiles_.size(); ++i) {
    auto& tile = tiles_[i];

    uint32_t x = cx_ - tile.j * size_ - size_;
    uint32_t y = cy_ - tile.i * size_ - size_;
    tile.x = x;
    tile.y = y;
    tile.size = size_;
  }
}

void TileSelectorWidget::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(0, 0, width(), height(), QBrush(Qt::white));

  painter.setPen(Qt::red);

  for (int32_t i = 0; i < int32_t(trajectory_.size()) - 1; ++i) {
    QPoint p1(cx_ - trajectory_[i].y() * size_, cy_ - trajectory_[i].x() * size_);
    QPoint p2(cx_ - trajectory_[i + 1].y() * size_, cy_ - trajectory_[i + 1].x() * size_);
    painter.drawLine(p1.x(), p1.y(), p2.x(), p2.y());
  }

  painter.setPen(Qt::black);

  for (uint32_t i = 0; i < tiles_.size(); ++i) {
    const auto& tile = tiles_[i];

    painter.drawRect(tile.x, tile.y, tile.size, tile.size);
  }

  // draw legend.
  int32_t i_current = -1;
  int32_t j_current = -1;
  if (selectedTile_ > -1) {
    const auto& tile = tiles_[selectedTile_];
    i_current = tile.i;
    j_current = tile.j;
  }

  for (uint32_t i = 0; i < numTilesY_; ++i) {
    uint32_t x = cx_ - (numTilesY_ - i - 1) * size_ - size_;
    uint32_t y = cy_ - (numTilesX_)*size_ - size_;

    painter.setPen(Qt::black);
    if (j_current == int32_t(numTilesY_ - i - 1)) painter.setPen(Qt::red);

    std::stringstream sstr;
    sstr << (char)(65 + i);
    painter.drawText(QRect(x, y, size_, size_), Qt::AlignCenter, QString::fromStdString(sstr.str()));
  }

  for (uint32_t j = 0; j < numTilesX_; ++j) {
    uint32_t x = cx_ - (numTilesY_)*size_ - size_;
    uint32_t y = cy_ - (numTilesX_ - j - 1) * size_ - size_;

    painter.setPen(Qt::black);
    if (i_current == int32_t(numTilesX_ - j - 1)) painter.setPen(Qt::red);

    painter.drawText(QRect(x, y, size_, size_), Qt::AlignCenter, QString::number(j + 1));
  }

  if (selectedTile_ > -1) {
    const auto& tile = tiles_[selectedTile_];
    painter.setPen(QPen(QColor(251, 187, 0), 2));
    painter.drawRect(tile.x, tile.y, tile.size, tile.size);
  }

  painter.end();
}

void TileSelectorWidget::setTrajectory(const std::vector<Eigen::Vector2f>& trajectory) { trajectory_ = trajectory; }
