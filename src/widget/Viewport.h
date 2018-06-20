/* \brief a view, which can be used to label points.
 *
 *
 */
#ifndef POINTVIEW_H_
#define POINTVIEW_H_

#include <stdint.h>
#include <list>
#include <vector>

#include <glow/glbase.h>

#include <QtCore/QTime>
#include <QtCore/QTimer>
#include <QtGui/QMouseEvent>
#include <QtOpenGL/QGLWidget>

#include <GL/gl.h>
#include <GL/glu.h>

#include <glow/GlBuffer.h>
#include <glow/GlColor.h>
#include <glow/GlFramebuffer.h>
#include <glow/GlProgram.h>
#include <glow/GlRenderbuffer.h>
#include <glow/GlShaderCache.h>
#include <glow/GlTexture.h>
#include <glow/GlVertexArray.h>
#include <glow/util/RoSeCamera.h>

#include "common.h"
#include "data/ProjectedPoint.h"
#include "data/ViewFrustum.h"
#include "data/geometry.h"

class Viewport : public QGLWidget {
  Q_OBJECT
 public:
  enum AXIS { XYZ, X, Y, Z };

  enum MODE { NONE, PAINT, POLYGON };

  enum FLAGS { FLAG_OVERWRITE = 1, FLAG_OTHER = 2 };

  Viewport(QWidget* parent = 0, Qt::WindowFlags f = 0);
  ~Viewport();

  void setMaximumScans(uint32_t numScans);

  void setPoints(const std::vector<PointcloudPtr>& points, std::vector<LabelsPtr>& labels);

  /** \brief update all labels with GPU labels. **/
  void updateLabels();

  void setLabelVisibility(uint32_t label, bool visible);

  void setDrawingOption(const std::string& name, bool value);

  void setMinRange(float range);
  void setMaxRange(float range);

  void setGroundRemoval(bool value);
  void setGroundThreshold(float value);

  void setScanIndex(uint32_t idx);

  void setTileInfo(float x, float y, float tileSize);

 signals:
  void labelingChanged();

 public slots:
  /** \brief set axis fixed **/
  void setFixedAxis(AXIS axis);

  void setRadius(float value);
  /** \brief label used when in PAINT **/
  void setLabel(uint32_t label);
  void setLabelColors(const std::map<uint32_t, glow::GlColor>& colors);
  void setPointSize(int value);

  void setMode(MODE mode);
  void setFlags(int32_t flags);
  void setOverwrite(bool value);

  void setFilteredLabels(const std::vector<uint32_t>& labels);

 protected:
  bool initContext() {
    // enabling core profile
    QGLFormat corefmt;
    corefmt.setVersion(5, 0);  // getting highest compatible format...
    corefmt.setProfile(QGLFormat::CoreProfile);
    setFormat(corefmt);

    // version info.
    QGLFormat fmt = this->format();
    std::cout << "OpenGL Context Version " << fmt.majorVersion() << "." << fmt.minorVersion() << " "
              << ((fmt.profile() == QGLFormat::CoreProfile) ? "core profile" : "compatibility profile") << std::endl;

    makeCurrent();
    glow::inititializeGLEW();

    return true;
  }

  void initPrograms();
  void initVertexBuffers();

  void initializeGL();
  void resizeGL(int width, int height);
  void paintGL();

  void mousePressEvent(QMouseEvent*);
  void mouseReleaseEvent(QMouseEvent*);
  void mouseMoveEvent(QMouseEvent*);
  void keyPressEvent(QKeyEvent*);

  glow::GlCamera::KeyboardModifier resolveKeyboardModifier(Qt::KeyboardModifiers modifiers);

  glow::GlCamera::MouseButton resolveMouseButton(Qt::MouseButtons button);

  //  void drawPoints(const std::vector<Point3f>& points, const std::vector<uint32_t>& labels);
  void labelPoints(int32_t x, int32_t y, float radius, uint32_t label);

  bool contextInitialized_;
  std::map<uint32_t, glow::GlColor> mLabelColors;

  std::vector<PointcloudPtr> points_;
  std::vector<LabelsPtr> labels_;

  glow::RoSeCamera mCamera;
  bool mChangeCamera{false};

  AXIS mAxis;
  MODE mMode;
  int32_t mFlags;

  uint32_t mCurrentLabel;
  float mRadius;
  std::vector<uint32_t> mFilteredLabels;

  /** selected endpoint **/

  bool buttonPressed;
  QTimer timer_;

  // shaders, etc.
  uint32_t maxScans_{50};
  uint32_t maxPointsPerScan_{150000};
  std::vector<Eigen::Matrix4f> bufPoses_;
  glow::GlBuffer<Point3f> bufPoints_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<float> bufRemissions_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<uint32_t> bufLabels_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<uint32_t> bufVisible_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};

  glow::GlTransformFeedback tfUpdateLabels_;
  glow::GlBuffer<uint32_t> bufUpdatedLabels_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};

  glow::GlTransformFeedback tfUpdateVisibility_;
  glow::GlBuffer<uint32_t> bufUpdatedVisiblity_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};

  glow::GlTextureRectangle texLabelColors_;

  glow::GlVertexArray vao_no_points_;
  glow::GlVertexArray vao_points_;
  glow::GlVertexArray vao_polygon_points_;
  glow::GlVertexArray vao_triangles_;

  glow::GlProgram prgDrawPose_;
  glow::GlProgram prgDrawPoints_;
  glow::GlProgram prgUpdateLabels_;
  glow::GlProgram prgUpdateVisibility_;
  glow::GlProgram prgPolygonPoints_;

  glow::GlFramebuffer fbMinimumHeightMap_;
  glow::GlTexture texMinimumHeightMap_;
  glow::GlProgram prgMinimumHeightMap_;

  int32_t pointSize_{1};

  glow::GlUniform<Eigen::Matrix4f> mvp_{"mvp", Eigen::Matrix4f::Identity()};
  glow::GlUniform<Eigen::Matrix4f> mvp_inv_t_{"mvp_inv_t", Eigen::Matrix4f::Identity()};

  Eigen::Matrix4f model_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f view_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f projection_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f conversion_{glow::RoSe2GL::matrix};

  struct BufferInfo {
    uint32_t index;
    uint32_t size;
  };

  std::map<Laserscan*, BufferInfo> bufferContent_;

  std::map<std::string, bool> drawingOption_;

  float minRange_{0.0f}, maxRange_{100.0f};

  bool removeGround_{false};
  float groundThreshold_{-1.6f};

  std::vector<glow::vec2> polygonPoints_;
  glow::GlBuffer<glow::vec2> bufPolygonPoints_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlTextureRectangle texTriangles_;
  glow::GlBuffer<glow::vec2> bufTriangles_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  uint32_t numTriangles_{0};

  uint32_t singleScanIdx_{0};

  glow::vec2 tilePos_;
  float tileSize_;
};

#endif /* POINTVIEW_H_ */
