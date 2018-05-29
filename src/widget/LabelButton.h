/** \brief a label button which can also be toggled. **/

#ifndef LABELBUTTON_H_
#define LABELBUTTON_H_

#include "../data/ColorGL.h"

#include <QtWidgets/QToolButton>

class LabelButton: public QToolButton
{
  Q_OBJECT
  public:
    LabelButton(QWidget* parent, const ColorGL& color);
    ~LabelButton();

    bool isHighlighted() const;
    bool wasHighlighted() const;

  signals:
    void highlighted(bool value);

  protected:
    virtual void mouseReleaseEvent(QMouseEvent * e);
    virtual void paintEvent(QPaintEvent* e);

    ColorGL color;
    bool mHighlighted, recentlyHighlighted;
};

#endif /* LABELBUTTON_H_ */
