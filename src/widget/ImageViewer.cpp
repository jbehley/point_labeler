#include "ImageViewer.h"

ImageViewer::ImageViewer(QWidget* parent, Qt::WindowFlags f) : QWidget(parent, f) {}

void ImageViewer::setImage(const std::string& filename) {
  currentImage_ = QPixmap(QString::fromStdString(filename));
  update();
}

void ImageViewer::paintEvent(QPaintEvent* event) {
  QPainter painter(this);

  painter.drawPixmap(0, 0, width(), height(), currentImage_);
}

void ImageViewer::resizeEvent(QResizeEvent* event) { update(); }
