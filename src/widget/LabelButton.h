#ifndef LABELBUTTON_H_
#define LABELBUTTON_H_

#include <QtWidgets/QToolButton>

/** \brief a label button which can also be toggled.
 *
 *  \author behley
 **/
class LabelButton : public QToolButton {
  Q_OBJECT
 public:
  LabelButton(QWidget* parent, const QString& name, const QColor& color);
  ~LabelButton();

  bool isHighlighted() const;
  bool wasHighlighted() const;

  void setHighlighted(bool value);

 signals:
  void highlighted(bool value);

 protected:
  virtual void mouseReleaseEvent(QMouseEvent* e);
  virtual void paintEvent(QPaintEvent* e);

  QString name_;
  QColor color_;
  bool mHighlighted, recentlyHighlighted;
};

#endif /* LABELBUTTON_H_ */
