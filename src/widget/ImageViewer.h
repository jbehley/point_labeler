#ifndef SRC_WIDGET_IMAGEVIEWER_H_
#define SRC_WIDGET_IMAGEVIEWER_H_

#include <QtGui/QMouseEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QWidget>

/** \brief show an image.
 *  \author behley
 **/
class ImageViewer : public QWidget {
  Q_OBJECT
 public:
  ImageViewer(QWidget* parent = 0, Qt::WindowFlags f = 0);

  void setImage(const std::string& filename);

 protected:
  void paintEvent(QPaintEvent* event);
  void resizeEvent(QResizeEvent* event);

  std::string imageFilename;
  QPixmap currentImage_;
};

#endif /* SRC_WIDGET_IMAGEVIEWER_H_ */
