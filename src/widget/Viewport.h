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
#include <glow/GlProgram.h>
#include <glow/GlShaderCache.h>
#include <glow/GlVertexArray.h>
#include <glow/util/RoSeCamera.h>

#include "data/ViewFrustum.h"
#include "data/geometry.h"

class Viewport : public QGLWidget {
  Q_OBJECT
 public:
  enum AXIS { XYZ, X, Y, Z };

  enum MODE { NONE, PAINT, CYLINDER };

  enum FLAGS { FLAG_OVERWRITE = 1, FLAG_OTHER = 2 };

  Viewport(QWidget* parent = 0, Qt::WindowFlags f = 0);
  ~Viewport();

 signals:
  void newCylinderFinished();
  void cylinderChanged();
  void labelingChanged();

 public slots:
  /** \brief set axis fixed **/
  void setFixedAxis(AXIS axis);
  void setPoints(const std::vector<Point3f>& points, std::vector<uint32_t>& labels);

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

  void drawPoints(const std::vector<Point3f>& points, const std::vector<uint32_t>& labels);
  void labelPoints(int32_t x, int32_t y, float radius, uint32_t label);
  void updateProjections();

  /** \brief get 3d coordinates of a click.
   *
   *  \return true, if valid point has been clicked, false otherwise.
   */
  //  bool getClickedPoint(const QPoint& mousePosition, Point3f& clickedPoint);

  bool contextInitialized_;
  std::map<uint32_t, glow::GlColor> mLabelColors;

  const std::vector<Point3f>* points;
  std::vector<uint32_t>* labels;
  std::vector<Point3f> projected_points;

  glow::RoSeCamera mCamera;

  AXIS mAxis;
  MODE mMode;
  int32_t mFlags;

  uint32_t mCurrentLabel;
  float mRadius;
  std::vector<uint32_t> mFilteredLabels;

  /** selected endpoint **/

  bool buttonPressed;
  ViewFrustum vf;
  QTimer timer_;

  // shaders, etc.
  glow::GlBuffer<glow::vec3> bufLabelColors{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<Point3f> bufPoints_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<uint32_t> bufLabels_{glow::BufferTarget::ELEMENT_ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};

  glow::GlVertexArray vao_no_points_;
  glow::GlVertexArray vao_points_;

  glow::GlProgram prgDrawPose_;
  glow::GlProgram prgDrawPoints_;

  int32_t pointSize_{1};
  std::map<std::string, bool> drawing_options_;

  glow::GlUniform<Eigen::Matrix4f> mvp_{"mvp", Eigen::Matrix4f::Identity()};
  glow::GlUniform<Eigen::Matrix4f> mvp_inv_t_{"mvp_inv_t", Eigen::Matrix4f::Identity()};

  Eigen::Matrix4f model_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f view_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f projection_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f conversion_{glow::RoSe2GL::matrix};
};

#endif /* POINTVIEW_H_ */
