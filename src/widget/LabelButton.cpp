#include "LabelButton.h"
#include <stdint.h>
#include <QtGui/QMouseEvent>
#include <QtGui/QPainter>

LabelButton::LabelButton(QWidget* parent, const ColorGL& c) :
  QToolButton(parent), color(c), mHighlighted(false), recentlyHighlighted(false)
{

}

LabelButton::~LabelButton()
{

}

bool LabelButton::isHighlighted() const
{
  return mHighlighted;
}

bool LabelButton::wasHighlighted() const
{
  return recentlyHighlighted;
}

void LabelButton::mouseReleaseEvent(QMouseEvent * e)
{
  recentlyHighlighted = false;

  if (e->modifiers() == Qt::ControlModifier)
  {
    mHighlighted = !mHighlighted;

    emit
    highlighted(mHighlighted);

    recentlyHighlighted = true;
  }


  QToolButton::mouseReleaseEvent(e);
  update();
}

void LabelButton::paintEvent(QPaintEvent* e)
{
  QToolButton::paintEvent(e);

  QPainter painter(this);
  int32_t w = width();
  int32_t h = height();

  QColor col(255.0f * color.R, 255.0f * color.G, 255.0f * color.B, 255);
  painter.fillRect(5, 5, w - 10, h - 10, QBrush(col, Qt::SolidPattern));

  if (mHighlighted)
  {
    QPen pen(Qt::blue, 2.0f, Qt::DashLine);
    painter.setPen(pen);
    painter.drawRect(0, 0, w, h);
  }
}
