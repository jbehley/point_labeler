#ifndef LABELBUTTON_H_
#define LABELBUTTON_H_

#include <glow/GlColor.h>

#include <QtWidgets/QToolButton>

/** \brief a label button which can also be toggled. **/
class LabelButton: public QToolButton
{
  Q_OBJECT
  public:
    LabelButton(QWidget* parent, const QColor& color);
    ~LabelButton();

    bool isHighlighted() const;
    bool wasHighlighted() const;

  signals:
    void highlighted(bool value);

  protected:
    virtual void mouseReleaseEvent(QMouseEvent * e);
    virtual void paintEvent(QPaintEvent* e);

    QColor color;
    bool mHighlighted, recentlyHighlighted;
};

#endif /* LABELBUTTON_H_ */
