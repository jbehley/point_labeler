#include "TileSelectorWidget.h"
#include <cmath>

TileSelectorWidget::TileSelectorWidget(QWidget* parent, Qt::WindowFlags f) {}

void TileSelectorWidget::initialize(const std::vector<KittiReader::Tile>& tiles, uint32_t numTilesX,
                                    uint32_t numTilesY) {
  tiles_.clear();

  numTilesX_ = numTilesX;
  numTilesY_ = numTilesY;

  size_ = float(width() - 1) / std::max(numTilesX_, numTilesY_);

  cx_ = width() - 0.5f * (width() - numTilesY * size_);
  cy_ = height() - 0.5f * (height() - numTilesX * size_);

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
  int32_t j = (cx_ - event->x()) / size_;
  int32_t i = (cy_ - event->y()) / size_;

  if (i < 0 || i >= int32_t(numTilesY_) || j < 0 || j >= int32_t(numTilesX_)) return;

  select(i, j);
}

void TileSelectorWidget::resizeEvent(QResizeEvent* event) {
  size_ = float(std::min(width(), height()) - 1) / std::max(numTilesX_, numTilesY_);

  cx_ = width() - 0.5f * (width() - numTilesY_ * size_);
  cy_ = height() - 0.5f * (height() - numTilesX_ * size_);

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
  painter.fillRect(0, 0, width(), height(), QBrush(Qt::white));
  painter.setPen(Qt::black);

  for (uint32_t i = 0; i < tiles_.size(); ++i) {
    const auto& tile = tiles_[i];

    painter.drawRect(tile.x, tile.y, tile.size, tile.size);
  }

  if (selectedTile_ > -1) {
    const auto& tile = tiles_[selectedTile_];
    painter.setPen(QPen(QColor(251, 187, 0), 2));
    painter.drawRect(tile.x, tile.y, tile.size, tile.size);
  }
  painter.end();
}
