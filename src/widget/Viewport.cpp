#include "Viewport.h"
#include "data/Math.h"
#include "data/draw_utils.h"
#include "data/misc.h"

#include <glow/ScopedBinder.h>
#include <glow/glutil.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include "rv/Stopwatch.h"

#include <glow/GlState.h>

using namespace glow;
using namespace rv;

Viewport::Viewport(QWidget* parent, Qt::WindowFlags f)
    : QGLWidget(parent, 0, f),
      contextInitialized_(initContext()),
      mAxis(XYZ),
      mMode(NONE),
      mFlags(FLAG_OVERWRITE),
      mCurrentLabel(0),
      mRadius(5),
      buttonPressed(false),
      texLabelColors_(256, 1, TextureFormat::RGB),
      fbMinimumHeightMap_(100, 100),
      texMinimumHeightMap_(100, 100, TextureFormat::R_FLOAT),
      texTriangles_(3 * 100, 1, TextureFormat::RGB) {
  connect(&timer_, &QTimer::timeout, [this]() { this->updateGL(); });

  //  setMouseTracking(true);

  drawingOption_["coordinate axis"] = true;

  conversion_ = RoSe2GL::matrix;

  uint32_t max_size = maxScans_ * maxPointsPerScan_;

  bufPoses_.resize(maxScans_);
  bufPoints_.resize(max_size);
  bufRemissions_.resize(max_size);
  bufVisible_.resize(max_size);
  bufLabels_.resize(max_size);

  bufPolygonPoints_.reserve(100);

  bufUpdatedLabels_.resize(maxPointsPerScan_);
  std::vector<std::string> update_varyings{"out_label"};
  tfUpdateLabels_.attach(update_varyings, bufUpdatedLabels_);

  bufUpdatedVisiblity_.resize(maxPointsPerScan_);
  std::vector<std::string> visibility_varyings{"out_visible"};
  tfUpdateVisibility_.attach(visibility_varyings, bufUpdatedVisiblity_);

  initPrograms();
  initVertexBuffers();

  drawingOption_["remission"] = true;
  drawingOption_["color"] = false;
  drawingOption_["single scan"] = false;
  drawingOption_["show all points"] = false;

  texLabelColors_.setMinifyingOperation(TexRectMinOp::NEAREST);
  texLabelColors_.setMagnifyingOperation(TexRectMagOp::NEAREST);

  texTriangles_.setMinifyingOperation(TexRectMinOp::NEAREST);
  texTriangles_.setMagnifyingOperation(TexRectMagOp::NEAREST);

  texMinimumHeightMap_.setMinifyingOperation(TexMinOp::NEAREST);
  texMinimumHeightMap_.setMagnifyingOperation(TexMagOp::NEAREST);

  fbMinimumHeightMap_.attach(FramebufferAttachment::COLOR0, texMinimumHeightMap_);
  GlRenderbuffer depthbuffer(fbMinimumHeightMap_.width(), fbMinimumHeightMap_.height(),
                             RenderbufferFormat::DEPTH_STENCIL);
  fbMinimumHeightMap_.attach(FramebufferAttachment::DEPTH_STENCIL, depthbuffer);

  setAutoFillBackground(false);

  glow::_CheckGlError(__FILE__, __LINE__);
}

Viewport::~Viewport() {}

void Viewport::initPrograms() {
  prgDrawPoints_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/draw_points.vert"));
  prgDrawPoints_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawPoints_.link();

  prgDrawPose_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/empty.vert"));
  prgDrawPose_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/draw_pose.geom"));
  prgDrawPose_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawPose_.link();

  prgUpdateLabels_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/update_labels.vert"));
  prgUpdateLabels_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/empty.frag"));
  prgUpdateLabels_.attach(tfUpdateLabels_);
  prgUpdateLabels_.link();

  prgUpdateVisibility_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/update_visibility.vert"));
  prgUpdateVisibility_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/empty.frag"));
  prgUpdateVisibility_.attach(tfUpdateVisibility_);
  prgUpdateVisibility_.link();

  prgPolygonPoints_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/draw_polygon.vert"));
  prgPolygonPoints_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgPolygonPoints_.link();

  prgMinimumHeightMap_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/gen_heightmap.vert"));
  prgMinimumHeightMap_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/gen_heightmap.frag"));
  prgMinimumHeightMap_.link();
}

void Viewport::initVertexBuffers() {
  vao_points_.setVertexAttribute(0, bufPoints_, 3, AttributeType::FLOAT, false, sizeof(Point3f), nullptr);
  vao_points_.setVertexAttribute(1, bufRemissions_, 1, AttributeType::FLOAT, false, sizeof(float), nullptr);
  vao_points_.setVertexAttribute(2, bufLabels_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);
  vao_points_.setVertexAttribute(3, bufVisible_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);

  vao_polygon_points_.setVertexAttribute(0, bufPolygonPoints_, 2, AttributeType::FLOAT, false, sizeof(vec2), nullptr);

  vao_triangles_.setVertexAttribute(0, bufTriangles_, 2, AttributeType::FLOAT, false, sizeof(vec2), nullptr);
}

/** \brief set axis fixed (x = 1, y = 2, z = 3) **/
void Viewport::setFixedAxis(AXIS axis) { mAxis = axis; }

void Viewport::setMaximumScans(uint32_t numScans) {
  maxScans_ = numScans;

  uint32_t max_size = maxScans_ * maxPointsPerScan_;

  bufPoses_.resize(maxScans_);
  bufPoints_.resize(max_size);
  bufRemissions_.resize(max_size);
  bufVisible_.resize(max_size);
  bufLabels_.resize(max_size);
}

void Viewport::setPoints(const std::vector<PointcloudPtr>& p, std::vector<LabelsPtr>& l) {
  std::cout << "Setting points..." << std::flush;

  glow::_CheckGlError(__FILE__, __LINE__);

  // FIXME: improve usage of resources:
  //   Use transform feedback to get points inside the tile.

  // determine which labels need to be updated.
  std::vector<uint32_t> indexes;
  index_difference(labels_, l, indexes);

  glow::GlBuffer<uint32_t> bufReadBuffer{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STREAM_READ};
  bufReadBuffer.resize(maxPointsPerScan_);

  for (auto index : indexes) {
    if (bufferContent_.find(points_[index].get()) == bufferContent_.end()) continue;

    const BufferInfo& info = bufferContent_[points_[index].get()];
    // replace label information with labels from GPU.
    // copy first to other buffer and read from that.
    bufLabels_.copyTo(info.index * maxPointsPerScan_, info.size, bufReadBuffer, 0);
    bufReadBuffer.get(*labels_[index], 0, info.size);
  }

  points_ = p;
  labels_ = l;
  glow::_CheckGlError(__FILE__, __LINE__);

  // TODO: on unload => fetch labels and update labels in file.

  {
    // first remove entries using a set_difference
    std::vector<Laserscan*> before;
    std::vector<Laserscan*> after;
    for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) before.push_back(it->first);
    for (auto it = p.begin(); it != p.end(); ++it) after.push_back(it->get());

    std::sort(before.begin(), before.end());
    std::sort(after.begin(), after.end());

    std::vector<Laserscan*> needsDelete(before.size());
    std::vector<Laserscan*>::iterator end =
        std::set_difference(before.begin(), before.end(), after.begin(), after.end(), needsDelete.begin());

    for (auto it = needsDelete.begin(); it != end; ++it) {
      bufferContent_.erase(*it);
    }
  }

  std::vector<int32_t> usedIndexes;
  for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) {
    usedIndexes.push_back(it->second.index);
  }

  std::sort(usedIndexes.begin(), usedIndexes.end());
  usedIndexes.push_back(maxScans_);

  std::vector<int32_t> freeIndexes;
  for (int32_t j = 0; j < usedIndexes[0]; ++j) freeIndexes.push_back(j);
  for (uint32_t i = 0; i < usedIndexes.size() - 1; ++i) {
    for (int32_t j = usedIndexes[i] + 1; j < usedIndexes[i + 1]; ++j) freeIndexes.push_back(j);
  }

  uint32_t nextFree = 0;
  uint32_t loadedScans = 0;
  float memcpy_time = 0.0f;

  Stopwatch::tic();
  for (uint32_t i = 0; i < points_.size(); ++i) {
    if (bufferContent_.find(points_[i].get()) == bufferContent_.end()) {
      if (nextFree == freeIndexes.size()) {
        std::cerr << "Warning: insufficient memory for scan." << std::endl;
        break;
      }

      int32_t index = freeIndexes[nextFree++];
      //      std::cout << index << std::endl;

      // not already loaded to buffer, need to transfer data to next free spot.
      uint32_t num_points = std::min(maxPointsPerScan_, points_[i]->size());
      if (points_[i]->size() >= maxPointsPerScan_) std::cerr << "warning: losing some points" << std::endl;

      std::vector<uint32_t> visible(num_points, 1);

      for (uint32_t j = 0; j < num_points; ++j) {
        if (std::find(mFilteredLabels.begin(), mFilteredLabels.end(), (*labels_[i])[j]) != mFilteredLabels.end()) {
          visible[j] = 0;
        }
      }

      Stopwatch::tic();

      BufferInfo info;
      info.index = index;
      info.size = num_points;

      bufferContent_[points_[i].get()] = info;
      bufPoses_[index] = points_[i]->pose;
      bufPoints_.replace(index * maxPointsPerScan_, &points_[i]->points[0], num_points);
      if (points_[i]->hasRemissions())
        bufRemissions_.replace(index * maxPointsPerScan_, &points_[i]->remissions[0], num_points);
      bufLabels_.replace(index * maxPointsPerScan_, &(*labels_[i])[0], num_points);
      bufVisible_.replace(index * maxPointsPerScan_, &visible[0], num_points);

      memcpy_time += Stopwatch::toc();

      loadedScans += 1;
    }
  }
  float total_time = Stopwatch::toc();

  glow::_CheckGlError(__FILE__, __LINE__);

  // generate height map.

  float groundResolution = 0.2f;
  uint32_t width = tileSize_ / groundResolution;
  uint32_t height = tileSize_ / groundResolution;

  if (fbMinimumHeightMap_.width() != width || fbMinimumHeightMap_.height() != height) {
    fbMinimumHeightMap_.resize(width, height);
    texMinimumHeightMap_.resize(width, height);

    // update also depth buffer.
    GlRenderbuffer depthbuffer(fbMinimumHeightMap_.width(), fbMinimumHeightMap_.height(),
                               RenderbufferFormat::DEPTH_STENCIL);
    fbMinimumHeightMap_.attach(FramebufferAttachment::COLOR0, texMinimumHeightMap_);
    fbMinimumHeightMap_.attach(FramebufferAttachment::DEPTH_STENCIL, depthbuffer);
  }

  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);

  glViewport(0, 0, fbMinimumHeightMap_.width(), fbMinimumHeightMap_.height());

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  fbMinimumHeightMap_.bind();
  prgMinimumHeightMap_.bind();
  vao_points_.bind();

  prgMinimumHeightMap_.setUniform(GlUniform<float>("minHeight", -3.0f));
  prgMinimumHeightMap_.setUniform(GlUniform<float>("maxHeight", 2.0f));

  prgMinimumHeightMap_.setUniform(GlUniform<float>("maxRange", maxRange_));
  prgMinimumHeightMap_.setUniform(GlUniform<float>("minRange", minRange_));

  prgMinimumHeightMap_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
  prgMinimumHeightMap_.setUniform(GlUniform<float>("tileSize", tileSize_));

  for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) {
    prgMinimumHeightMap_.setUniform(GlUniform<Eigen::Matrix4f>("pose", it->first->pose));

    glDrawArrays(GL_POINTS, it->second.index * maxPointsPerScan_, it->second.size);
  }

  vao_points_.release();
  prgMinimumHeightMap_.release();
  fbMinimumHeightMap_.release();

  glViewport(vp[0], vp[1], vp[2], vp[3]);

  glDepthFunc(GL_LEQUAL);

  //  std::vector<float> data;
  //  texMinimumHeightMap_.download(data);
  ////  std::ofstream out("texture.txt");
  ////  for (auto v : data) out << v << ", ";
  ////  out << std::endl;
  ////  out.close();
  glow::_CheckGlError(__FILE__, __LINE__);

  std::cout << "Loaded " << loadedScans << " of total " << points_.size() << " scans." << std::endl;
  std::cout << "memcpy: " << memcpy_time << " s / " << total_time << " s." << std::endl;

  updateGL();
}

void Viewport::updateLabels() {
  glow::_CheckGlError(__FILE__, __LINE__);

  glow::GlBuffer<uint32_t> bufReadBuffer{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STREAM_READ};
  bufReadBuffer.resize(maxPointsPerScan_);

  for (uint32_t i = 0; i < points_.size(); ++i) {
    if (bufferContent_.find(points_[i].get()) == bufferContent_.end()) continue;

    const BufferInfo& info = bufferContent_[points_[i].get()];
    // replace label information with labels from GPU.
    // copy first to other buffer and read from that.
    bufLabels_.copyTo(info.index * maxPointsPerScan_, info.size, bufReadBuffer, 0);
    bufReadBuffer.get(*labels_[i], 0, info.size);
  }

  glow::_CheckGlError(__FILE__, __LINE__);
}

void Viewport::setRadius(float value) { mRadius = value; }

void Viewport::setLabel(uint32_t label) { mCurrentLabel = label; }

void Viewport::setLabelColors(const std::map<uint32_t, glow::GlColor>& colors) {
  mLabelColors = colors;

  std::vector<uint8_t> label_colors(3 * 256);
  for (auto it = mLabelColors.begin(); it != mLabelColors.end(); ++it) {
    label_colors[3 * it->first] = it->second.R * 255;
    label_colors[3 * it->first + 1] = it->second.G * 255;
    label_colors[3 * it->first + 2] = it->second.B * 255;
  }
  texLabelColors_.assign(PixelFormat::RGB, PixelType::UNSIGNED_BYTE, &label_colors[0]);
}

void Viewport::setPointSize(int value) {
  pointSize_ = value;
  updateGL();
}

void Viewport::setMode(MODE mode) {
  mMode = mode;

  updateGL();
}

void Viewport::setFlags(int32_t flags) { mFlags = flags; }

void Viewport::setOverwrite(bool value) {
  if (value)
    mFlags = mFlags | FLAG_OVERWRITE;
  else
    mFlags = mFlags & ~FLAG_OVERWRITE;
}

void Viewport::setDrawingOption(const std::string& name, bool value) {
  drawingOption_[name] = value;
  updateGL();
}

void Viewport::setMinRange(float range) { minRange_ = range; }

void Viewport::setMaxRange(float range) { maxRange_ = range; }

void Viewport::setFilteredLabels(const std::vector<uint32_t>& labels) {
  std::vector<uint32_t> labels_before = mFilteredLabels;
  mFilteredLabels = labels;
  std::sort(mFilteredLabels.begin(), mFilteredLabels.end());

  std::vector<uint32_t> difference(std::max(labels_before.size(), labels.size()));
  auto end = std::set_difference(labels_before.begin(), labels_before.end(), mFilteredLabels.begin(),
                                 mFilteredLabels.end(), difference.begin());

  for (auto it = difference.begin(); it != end; ++it) setLabelVisibility(*it, 1);  // now visible again.

  end = std::set_difference(mFilteredLabels.begin(), mFilteredLabels.end(), labels_before.begin(), labels_before.end(),
                            difference.begin());

  for (auto it = difference.begin(); it != end; ++it) setLabelVisibility(*it, 0);  // now invisible.

  updateGL();
}

void Viewport::setGroundRemoval(bool value) {
  removeGround_ = value;
  updateGL();
}

void Viewport::setGroundThreshold(float value) {
  groundThreshold_ = value;
  updateGL();
}

void Viewport::setScanIndex(uint32_t idx) {
  singleScanIdx_ = idx;
  updateGL();
}

void Viewport::setLabelVisibility(uint32_t label, bool visible) {
  ScopedBinder<GlVertexArray> vaoBinder(vao_points_);
  ScopedBinder<GlProgram> programBinder(prgUpdateVisibility_);
  ScopedBinder<GlTransformFeedback> feedbackBinder(tfUpdateVisibility_);

  prgUpdateVisibility_.setUniform(GlUniform<uint32_t>("label", label));
  prgUpdateVisibility_.setUniform(GlUniform<uint32_t>("visibility", visible ? 1 : 0));

  glEnable(GL_RASTERIZER_DISCARD);

  for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) {
    tfUpdateVisibility_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, it->second.index * maxPointsPerScan_, it->second.size);
    tfUpdateVisibility_.end();

    bufUpdatedVisiblity_.copyTo(bufVisible_, it->second.index * maxPointsPerScan_);
  }

  glDisable(GL_RASTERIZER_DISCARD);
}

void Viewport::initializeGL() {
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_LINE_SMOOTH);

  mCamera.lookAt(5.0f, 5.0f, 5.0f, 0.0f, 0.0f, 0.0f);
}

void Viewport::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);

  // set projection matrix
  float fov = radians(45.0f);
  float aspect = float(w) / float(h);

  projection_ = glPerspective(fov, aspect, 0.1f, 2000.0f);
}

void Viewport::paintGL() {
  glow::_CheckGlError(__FILE__, __LINE__);

  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPointSize(pointSize_);

  model_ = Eigen::Matrix4f::Identity();
  view_ = mCamera.matrix();

  mvp_ = projection_ * view_ * conversion_;

  if (drawingOption_["coordinate axis"]) {
    // coordinateSytem_->pose = Eigen::Matrix4f::Identity();
    ScopedBinder<GlProgram> program_binder(prgDrawPose_);
    ScopedBinder<GlVertexArray> vao_binder(vao_no_points_);

    prgDrawPose_.setUniform(mvp_);
    prgDrawPose_.setUniform(GlUniform<Eigen::Matrix4f>("pose", Eigen::Matrix4f::Identity()));
    prgDrawPose_.setUniform(GlUniform<float>("size", 1.0f));

    glDrawArrays(GL_POINTS, 0, 1);
  }

  if (points_.size() > 0) {
    glPointSize(pointSize_);

    ScopedBinder<GlProgram> program_binder(prgDrawPoints_);
    ScopedBinder<GlVertexArray> vao_binder(vao_points_);

    prgDrawPoints_.setUniform(GlUniform<bool>("useRemission", drawingOption_["remission"]));
    prgDrawPoints_.setUniform(GlUniform<bool>("useColor", drawingOption_["color"]));
    prgDrawPoints_.setUniform(GlUniform<float>("maxRange", maxRange_));
    prgDrawPoints_.setUniform(GlUniform<float>("minRange", minRange_));
    prgDrawPoints_.setUniform(GlUniform<bool>("removeGround", removeGround_));
    prgDrawPoints_.setUniform(GlUniform<float>("groundThreshold", groundThreshold_));
    prgDrawPoints_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
    prgDrawPoints_.setUniform(GlUniform<float>("tileSize", tileSize_));
    prgDrawPoints_.setUniform(GlUniform<bool>("showAllPoints", drawingOption_["show all points"]));
    prgDrawPoints_.setUniform(GlUniform<int32_t>("heightMap", 1));

    bool showSingleScan = drawingOption_["single scan"];

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.bind();

    glActiveTexture(GL_TEXTURE1);
    texMinimumHeightMap_.bind();

    for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) {
      if (showSingleScan && (it->first != points_[singleScanIdx_].get())) continue;
      prgDrawPoints_.setUniform(GlUniform<Eigen::Matrix4f>("pose", it->first->pose));

      mvp_ = projection_ * view_ * conversion_ * it->first->pose;
      prgDrawPoints_.setUniform(mvp_);

      glDrawArrays(GL_POINTS, it->second.index * maxPointsPerScan_, it->second.size);
    }

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.release();
    glActiveTexture(GL_TEXTURE1);
    texMinimumHeightMap_.release();
  }

  glDisable(GL_DEPTH_TEST);
  // Important: QPainter is apparently not working with OpenGL Core Profile < Qt5.9!!!!
  //  http://blog.qt.io/blog/2017/01/27/opengl-core-profile-context-support-qpainter/
  //  QPainter painter(this); // << does not work with OpenGL Core Profile.
  if (mMode == POLYGON) {
    ScopedBinder<GlProgram> program_binder(prgPolygonPoints_);

    vao_polygon_points_.bind();

    prgPolygonPoints_.setUniform(GlUniform<int32_t>("width", width()));
    prgPolygonPoints_.setUniform(GlUniform<int32_t>("height", height()));
    prgPolygonPoints_.setUniform(GlUniform<uint32_t>("label", mCurrentLabel));

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.bind();

    glDrawArrays(GL_LINE_LOOP, 0, bufPolygonPoints_.size());

    glPointSize(7.0f);

    glDrawArrays(GL_POINTS, 0, bufPolygonPoints_.size());
    vao_polygon_points_.release();

    //    vao_triangles_.bind();
    //
    //    glDrawArrays(GL_LINES, 0, bufTriangles_.size());
    //
    //    vao_triangles_.release();

    texLabelColors_.release();
  }

  glow::_CheckGlError(__FILE__, __LINE__);
}

std::ostream& operator<<(std::ostream& os, const vec2& v) {
  os << "(" << v.x << ", " << v.y << ")";
  return os;
}

void Viewport::mousePressEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.

  mChangeCamera = false;

  if (event->modifiers() == Qt::ControlModifier || (mMode == NONE)) {
    if (mCamera.mousePressed(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                             resolveKeyboardModifier(event->modifiers()))) {
      timer_.start(1. / 30.);
      mChangeCamera = true;
      polygonPoints_.clear();  // start over again.#
      bufPolygonPoints_.assign(polygonPoints_);
      bufTriangles_.resize(0);
      return;
    }
  } else if (mMode == PAINT) {
    buttonPressed = true;
    mChangeCamera = false;
    if (event->buttons() & Qt::LeftButton)
      labelPoints(event->x(), event->y(), mRadius, mCurrentLabel);
    else if (event->buttons() & Qt::RightButton)
      labelPoints(event->x(), event->y(), mRadius, 0);

    updateGL();
  } else if (mMode == POLYGON) {
    if (event->buttons() & Qt::LeftButton) {
      polygonPoints_.push_back(vec2(event->x(), event->y()));

    } else if (event->buttons() & Qt::RightButton) {
      if (polygonPoints_.size() > 2) {
        // finish polygon and label points.

        // 1. determine winding: https://blog.element84.com/polygon-winding-post.html

        std::vector<vec2> points = polygonPoints_;
        for (uint32_t i = 0; i < points.size(); ++i) {
          points[i].y = height() - points[i].y;  // flip y.
        }

        float winding = 0.0f;

        // important: take also last edge into account!
        for (uint32_t i = 0; i < points.size(); ++i) {
          const auto& p = points[(i + 1) % points.size()];
          const auto& q = points[i];

          winding += (p.x - q.x) * (q.y + p.y);
        }

        // invert  order if CW order.
        if (winding > 0) std::reverse(points.begin(), points.end());

        //        if (winding > 0) std::cout << "winding: CW" << std::endl;
        //        else std::cout << "winding: CCW" << std::endl;

        std::vector<Triangle> triangles;
        //        std::vector<glow::vec2> tris_verts;

        triangulate(points, triangles);

        //        std::cout << "#triangles: " << triangles.size() << std::endl;

        std::vector<vec3> texContent(3 * 100);
        for (uint32_t i = 0; i < triangles.size(); ++i) {
          auto t = triangles[i];
          texContent[3 * i + 0] = vec3(t.i.x / width(), (height() - t.i.y) / height(), 0);
          texContent[3 * i + 1] = vec3(t.j.x / width(), (height() - t.j.y) / height(), 0);
          texContent[3 * i + 2] = vec3(t.k.x / width(), (height() - t.k.y) / height(), 0);

          //          tris_verts.push_back(vec2(t.i.x, height() - t.i.y));
          //          tris_verts.push_back(vec2(t.j.x, height() - t.j.y));
          //          tris_verts.push_back(vec2(t.j.x, height() - t.j.y));
          //          tris_verts.push_back(vec2(t.k.x, height() - t.k.y));
          //          tris_verts.push_back(vec2(t.k.x, height() - t.k.y));
          //          tris_verts.push_back(vec2(t.i.x, height() - t.i.y));
        }

        numTriangles_ = triangles.size();
        // note: colors are in range [0,1] for FLOAT!
        texTriangles_.assign(PixelFormat::RGB, PixelType::FLOAT, &texContent[0]);
        //        bufTriangles_.assign(tris_verts);

        labelPoints(event->x(), event->y(), 0, mCurrentLabel);
      }

      polygonPoints_.clear();
    }

    bufPolygonPoints_.assign(polygonPoints_);

    repaint();
  }

  event->accept();
}

void Viewport::mouseReleaseEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (mChangeCamera) {
    timer_.stop();
    updateGL();
    if (mCamera.mouseReleased(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                              resolveKeyboardModifier(event->modifiers()))) {
      timer_.stop();
      updateGL();  // get the last action.

      return;
    }
  } else if (mMode == PAINT) {
    buttonPressed = false;

    if (event->buttons() & Qt::LeftButton)
      labelPoints(event->x(), event->y(), mRadius, mCurrentLabel);
    else if (event->buttons() & Qt::RightButton)
      labelPoints(event->x(), event->y(), mRadius, 0);

    updateGL();
  } else if (mMode == POLYGON) {
    if (polygonPoints_.size() > 0) {
      polygonPoints_.back().x = event->x();
      polygonPoints_.back().y = event->y();

      bufPolygonPoints_.assign(polygonPoints_);
    }

    repaint();
  }

  event->accept();
}

void Viewport::mouseMoveEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (mChangeCamera) {
    if (mCamera.mouseMoved(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                           resolveKeyboardModifier(event->modifiers()))) {
      return;
    }
  } else if (mMode == PAINT) {
    if (buttonPressed) {
      if (event->buttons() & Qt::LeftButton)
        labelPoints(event->x(), event->y(), mRadius, mCurrentLabel);
      else
        labelPoints(event->x(), event->y(), mRadius, 0);
    }
    updateGL();
  } else if (mMode == POLYGON) {
    if (polygonPoints_.size() > 0) {
      polygonPoints_.back().x = event->x();
      polygonPoints_.back().y = event->y();

      bufPolygonPoints_.assign(polygonPoints_);

      repaint();
    }
  }

  event->accept();
}

void Viewport::keyPressEvent(QKeyEvent* event) { event->ignore(); }

void Viewport::setTileInfo(float x, float y, float tileSize) {
  std::cout << x << ", " << y << ", " << tileSize << std::endl;
  tilePos_ = vec2(x, y);
  tileSize_ = tileSize;
}

void Viewport::labelPoints(int32_t x, int32_t y, float radius, uint32_t new_label) {
  if (points_.size() == 0 || labels_.size() == 0) return;

  //  std::cout << "called labelPoints(" << x << ", " << y << ", " << radius << ", " << new_label << ")" << std::flush;
  //  Stopwatch::tic();

  bool showSingleScan = drawingOption_["single scan"];

  ScopedBinder<GlVertexArray> vaoBinder(vao_points_);
  ScopedBinder<GlProgram> programBinder(prgUpdateLabels_);
  ScopedBinder<GlTransformFeedback> feedbackBinder(tfUpdateLabels_);

  prgUpdateLabels_.setUniform(GlUniform<vec2>("window_pos", glow::vec2(x, y)));
  prgUpdateLabels_.setUniform(GlUniform<int32_t>("width", width()));
  prgUpdateLabels_.setUniform(GlUniform<int32_t>("height", height()));
  prgUpdateLabels_.setUniform(GlUniform<float>("radius", radius));
  prgUpdateLabels_.setUniform(GlUniform<uint32_t>("new_label", new_label));
  prgUpdateLabels_.setUniform(GlUniform<bool>("overwrite", mFlags & FLAG_OVERWRITE));
  prgUpdateLabels_.setUniform(GlUniform<float>("maxRange", maxRange_));
  prgUpdateLabels_.setUniform(GlUniform<float>("minRange", minRange_));
  prgUpdateLabels_.setUniform(GlUniform<bool>("removeGround", removeGround_));
  prgUpdateLabels_.setUniform(GlUniform<float>("groundThreshold", groundThreshold_));
  prgUpdateLabels_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
  prgUpdateLabels_.setUniform(GlUniform<float>("tileSize", tileSize_));
  prgUpdateLabels_.setUniform(GlUniform<bool>("showAllPoints", drawingOption_["show all points"]));
  prgUpdateLabels_.setUniform(GlUniform<int32_t>("heightMap", 1));

  if (mMode == Viewport::PAINT) prgUpdateLabels_.setUniform(GlUniform<int32_t>("labelingMode", 0));
  if (mMode == Viewport::POLYGON) {
    prgUpdateLabels_.setUniform(GlUniform<int32_t>("labelingMode", 1));
    prgUpdateLabels_.setUniform(GlUniform<int32_t>("numTriangles", numTriangles_));
  }

  glActiveTexture(GL_TEXTURE0);
  texTriangles_.bind();

  glActiveTexture(GL_TEXTURE1);
  texMinimumHeightMap_.bind();

  glEnable(GL_RASTERIZER_DISCARD);

  uint32_t numIters = 0;
  for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) {
    if (showSingleScan && (it->first != points_[singleScanIdx_].get())) continue;
    mvp_ = projection_ * mCamera.matrix() * conversion_ * it->first->pose;

    prgUpdateLabels_.setUniform(mvp_);
    prgUpdateLabels_.setUniform(GlUniform<Eigen::Matrix4f>("pose", it->first->pose));

    tfUpdateLabels_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, it->second.index * maxPointsPerScan_, it->second.size);
    tfUpdateLabels_.end();

    bufUpdatedLabels_.copyTo(bufLabels_, it->second.index * maxPointsPerScan_);
    numIters += 1;
  }

  //  std::cout << numIters << " iters took " << std::flush;

  glDisable(GL_RASTERIZER_DISCARD);

  glActiveTexture(GL_TEXTURE0);
  texTriangles_.release();
  glActiveTexture(GL_TEXTURE1);
  texMinimumHeightMap_.release();
  //  std::cout << Stopwatch::toc() << " s." << std::endl;

  emit labelingChanged();
}

glow::GlCamera::KeyboardModifier Viewport::resolveKeyboardModifier(Qt::KeyboardModifiers modifiers) {
  // currently only single button presses are supported.
  GlCamera::KeyboardModifier modifier = GlCamera::KeyboardModifier::None;

  if (modifiers & Qt::ControlModifier)
    modifier = GlCamera::KeyboardModifier::CtrlDown;
  else if (modifiers & Qt::ShiftModifier)
    modifier = GlCamera::KeyboardModifier::ShiftDown;
  else if (modifiers & Qt::AltModifier)
    modifier = GlCamera::KeyboardModifier::AltDown;

  return modifier;
}

glow::GlCamera::MouseButton Viewport::resolveMouseButton(Qt::MouseButtons button) {
  // currently only single button presses are supported.
  GlCamera::MouseButton btn = GlCamera::MouseButton::NoButton;

  if (button & Qt::LeftButton)
    btn = GlCamera::MouseButton::LeftButton;
  else if (button & Qt::RightButton)
    btn = GlCamera::MouseButton::RightButton;
  else if (button & Qt::MiddleButton)
    btn = GlCamera::MouseButton::MiddleButton;

  return btn;
}
