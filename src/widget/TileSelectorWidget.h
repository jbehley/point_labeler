#ifndef SRC_WIDGET_TILESELECTORWIDGET_H_
#define SRC_WIDGET_TILESELECTORWIDGET_H_

#include <QtCore/QPoint>
#include <QtGui/QMouseEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QWidget>

#include "KittiReader.h"


/** \brief simple widget showing tiles corresponding to parts of the point cloud.
 *
 *  Note that the coordinate system is the normal robotic coordinate system.
 *  x axis is pointing upwards, y axis is pointing left.
 *
 *  \author behley
 */
class TileSelectorWidget : public QWidget {
  Q_OBJECT
 public:
  TileSelectorWidget(QWidget* parent = 0, Qt::WindowFlags f = 0);

  virtual ~TileSelectorWidget() {}

  /** \brief initialize tile view with given tiles. **/
  void initialize(const std::vector<KittiReader::Tile>& tiles, uint32_t numTilesX, uint32_t numTilesY);

  /** \brief select tile at (i,j) and emit signal. **/
  void select(uint32_t i, uint32_t j);

  /** \brief set selected tile without emitting signal. **/
  void setSelected(uint32_t i, uint32_t j);

  /** \brief set trajectory in tile coordinates. **/
  void setTrajectory(const std::vector<Eigen::Vector2f>& trajectory);

 signals:
  void tileSelected(int32_t i, int32_t j);

 protected:
  void mouseMoveEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);

  void paintEvent(QPaintEvent* event);
  void resizeEvent(QResizeEvent* event);

  struct TileProxy {
    TileProxy(uint32_t x, uint32_t y, uint32_t size) : x(x), y(y), size(size) {}

    uint32_t i{0}, j{0};
    uint32_t x, y;
    uint32_t size;

    bool selected{false};
  };

  std::vector<TileProxy> tiles_;
  int32_t selectedTile_{-1};
  uint32_t cx_{0}, cy_{0};
  uint32_t size_{0};
  uint32_t numTilesX_{0}, numTilesY_{0};
  std::vector<Eigen::Vector2f> trajectory_;
};

#endif /* SRC_WIDGET_TILESELECTORWIDGET_H_ */
