#include "LabelButton.h"
#include <stdint.h>
#include <QtGui/QMouseEvent>
#include <QtGui/QPainter>
#include <iostream>

LabelButton::LabelButton(QWidget* parent, const QString& name, const QColor& c)
    : QToolButton(parent), name_(name), color_(c), mHighlighted(false), recentlyHighlighted(false) {
  setCheckable(true);
  setMinimumSize(30, 30);
  setMaximumHeight(30);
  setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
}

LabelButton::~LabelButton() {}

bool LabelButton::isHighlighted() const { return mHighlighted; }

bool LabelButton::wasHighlighted() const { return recentlyHighlighted; }

void LabelButton::mouseReleaseEvent(QMouseEvent* e) {
  recentlyHighlighted = false;

  if (e->modifiers() == Qt::ControlModifier) {
    mHighlighted = !mHighlighted;

    emit highlighted(mHighlighted);

    recentlyHighlighted = true;
  }

  QToolButton::mouseReleaseEvent(e);
  update();
}

void LabelButton::setHighlighted(bool value) {
  mHighlighted = value;
  update();
}

void LabelButton::paintEvent(QPaintEvent* e) {
  QToolButton::paintEvent(e);

  QPainter painter(this);
  int32_t w = width();
  int32_t h = height();

  painter.fillRect(5, 5, w - 10, h - 10, QBrush(color_, Qt::SolidPattern));

  if (color_.toHsv().value() < 200) painter.setPen(Qt::white);

  int32_t name_width = painter.fontMetrics().boundingRect(name_).width();
  if (std::max(name_width, 50) < w - 10) painter.drawText(5, 5, w - 10, h - 10, Qt::AlignCenter, name_);

  if (mHighlighted) {
    QPen pen(Qt::blue, 2.0f, Qt::DashLine);
    painter.setPen(pen);
    painter.drawRect(0, 0, w, h);
  }

  painter.end();
}
