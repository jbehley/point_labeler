#ifndef SRC_WIDGET_TILESELECTORWIDGET_H_
#define SRC_WIDGET_TILESELECTORWIDGET_H_

#include <QtGui/QMouseEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QWidget>

#include "KittiReader.h"

class TileSelectorWidget : public QWidget {
  Q_OBJECT
 public:
  TileSelectorWidget(QWidget* parent = 0, Qt::WindowFlags f = 0);

  virtual ~TileSelectorWidget() {}

  void initialize(const std::vector<KittiReader::Tile>& tiles, uint32_t numTilesX, uint32_t numTilesY);

  /** \brief select tile at (i,j) **/
  void select(uint32_t i, uint32_t j);

  /** \brief set selected tile without emitting signal. **/
  void setSelected(uint32_t i, uint32_t j);

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
};

#endif /* SRC_WIDGET_TILESELECTORWIDGET_H_ */
