#include "Viewport.h"
#include "data/Math.h"
#include "data/draw_utils.h"
#include "data/misc.h"

#include <glow/GlCapabilities.h>
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
      texLabelColors_(1024, 1, TextureFormat::RGB),
      fbMinimumHeightMap_(100, 100),
      texMinimumHeightMap_(100, 100, TextureFormat::R_FLOAT),
      texTriangles_(3 * 100, 1, TextureFormat::RGB) {
  connect(&timer_, &QTimer::timeout, [this]() { this->updateGL(); });

  //  setMouseTracking(true);

  drawingOption_["coordinate axis"] = true;

  conversion_ = RoSe2GL::matrix;

  // FIXME: make max_size depending on memory. (have setting like AvailableMemory that specifies how much memory should
  // be used by program!)
  uint32_t max_size = maxScans_ * maxPointsPerScan_;

  bufPoses_.resize(maxScans_);
  bufPoints_.resize(max_size);
  bufVisible_.resize(max_size);
  bufLabels_.resize(max_size);
  bufScanIndexes_.resize(max_size);

  bufTempPoints_.reserve(maxPointsPerScan_);
  bufTempRemissions_.reserve(maxPointsPerScan_);
  bufTempLabels_.reserve(maxPointsPerScan_);
  bufTempVisible_.reserve(maxPointsPerScan_);

  uint32_t tempMem = bufTempPoints_.memorySize();
  tempMem += bufTempRemissions_.memorySize();
  tempMem += bufTempLabels_.memorySize();
  tempMem += bufTempVisible_.memorySize();

  std::cout << "temp mem size: " << float(tempMem) / (1000 * 1000) << " MB" << std::endl;

  bufPolygonPoints_.reserve(100);

  bufUpdatedLabels_.resize(maxPointsPerScan_);
  tfUpdateLabels_.attach({"out_label"}, bufUpdatedLabels_);

  bufUpdatedVisiblity_.resize(maxPointsPerScan_);
  tfUpdateVisibility_.attach({"out_visible"}, bufUpdatedVisiblity_);

  tfFillTilePoints_.attach({"out_point"}, bufPoints_);
  tfFillTilePoints_.attach({"out_label"}, bufLabels_);
  tfFillTilePoints_.attach({"out_visible"}, bufVisible_);
  tfFillTilePoints_.attach({"out_scanindex"}, bufScanIndexes_);

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

Viewport::~Viewport() {
  // workaround for strange behaviour with my Nvidia 860m. Even though the buffers fit into memory, they mess up
  // something up. However, I noticed that it does not cause havoc if the buffers are empty or small.

  bufPoints_.assign(std::vector<vec4>());
  bufLabels_.assign(std::vector<uint32_t>());
  bufScanIndexes_.assign(std::vector<vec2>());
  bufVisible_.assign(std::vector<uint32_t>());
}

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

  prgFillTilePoints_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/fill_tile_points.vert"));
  prgFillTilePoints_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/fill_tile_points.geom"));
  prgFillTilePoints_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/empty.frag"));
  prgFillTilePoints_.attach(tfFillTilePoints_);
  prgFillTilePoints_.link();

  prgDrawFrustum_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/empty.vert"));
  prgDrawFrustum_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/draw_frustum.geom"));
  prgDrawFrustum_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawFrustum_.link();

  glow::_CheckGlError(__FILE__, __LINE__);
}

void Viewport::initVertexBuffers() {
  vao_points_.setVertexAttribute(0, bufPoints_, 4, AttributeType::FLOAT, false, sizeof(glow::vec4), nullptr);
  vao_points_.setVertexAttribute(1, bufLabels_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);
  vao_points_.setVertexAttribute(2, bufVisible_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);

  vao_temp_points_.setVertexAttribute(0, bufTempPoints_, 3, AttributeType::FLOAT, false, sizeof(Point3f), nullptr);
  vao_temp_points_.setVertexAttribute(1, bufTempRemissions_, 1, AttributeType::FLOAT, false, sizeof(float), nullptr);
  vao_temp_points_.setVertexAttribute(2, bufTempLabels_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t),
                                      nullptr);
  vao_temp_points_.setVertexAttribute(3, bufTempVisible_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t),
                                      nullptr);

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
  bufVisible_.resize(max_size);
  bufLabels_.resize(max_size);
  bufScanIndexes_.resize(max_size);

  uint32_t memTile = 0;  // = bufPoses_.memorySize();
  memTile += bufPoints_.memorySize();
  memTile += bufVisible_.memorySize();
  memTile += bufLabels_.memorySize();
  memTile += bufScanIndexes_.memorySize();

  std::cout << "mem size: " << float(memTile) / (1000 * 1000) << " MB" << std::endl;
}

void Viewport::setPoints(const std::vector<PointcloudPtr>& p, std::vector<LabelsPtr>& l) {
  std::cout << "Setting points..." << std::flush;

  glow::_CheckGlError(__FILE__, __LINE__);

  points_ = p;
  labels_ = l;

  //  Stopwatch::tic();

  {
    ScopedBinder<GlVertexArray> vaoBinder(vao_temp_points_);
    ScopedBinder<GlProgram> programBinder(prgFillTilePoints_);
    ScopedBinder<GlTransformFeedback> feedbackBinder(tfFillTilePoints_);

    glow::_CheckGlError(__FILE__, __LINE__);

    prgFillTilePoints_.setUniform(GlUniform<float>("maxRange", maxRange_));
    prgFillTilePoints_.setUniform(GlUniform<float>("minRange", minRange_));
    prgFillTilePoints_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
    prgFillTilePoints_.setUniform(GlUniform<float>("tileSize", tileSize_));
    prgFillTilePoints_.setUniform(GlUniform<float>("tileBoundary", tileBoundary_));

    glEnable(GL_RASTERIZER_DISCARD);

    tfFillTilePoints_.begin(TransformFeedbackMode::POINTS);

    for (uint32_t t = 0; t < points_.size(); ++t) {
      prgFillTilePoints_.setUniform(GlUniform<float>("maxRange", maxRange_));
      prgFillTilePoints_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[t]->pose));

      uint32_t num_points = points_[t]->size();

      std::vector<uint32_t> visible(num_points, 1);

      for (uint32_t i = 0; i < num_points; ++i) {
        if (std::find(mFilteredLabels.begin(), mFilteredLabels.end(), (*labels_[t])[i]) != mFilteredLabels.end()) {
          visible[i] = 0;
        }
      }

      // copy data from CPU -> GPU.
      bufTempPoints_.assign(points_[t]->points);
      if (points_[t]->hasRemissions())
        bufTempRemissions_.assign(points_[t]->remissions);
      else
        bufTempRemissions_.assign(std::vector<float>(points_[t]->size(), 1.0f));
      bufTempLabels_.assign(*(labels_[t]));
      bufTempVisible_.assign(visible);

      prgFillTilePoints_.setUniform(GlUniform<uint32_t>("scan", t));

      // extract tile points.
      glDrawArrays(GL_POINTS, 0, points_[t]->size());
    }

    uint32_t numCopiedPoints = tfFillTilePoints_.end();

    glDisable(GL_RASTERIZER_DISCARD);

    bufPoints_.resize(numCopiedPoints);
    bufLabels_.resize(numCopiedPoints);
    bufVisible_.resize(numCopiedPoints);
    bufScanIndexes_.resize(numCopiedPoints);

    // get per scan information from  scan indexes.
    scanInfos_.clear();
    scanInfos_.resize(points_.size());
    for (auto& info : scanInfos_) {
      info.start = 0;
      info.size = 0;
    }

    glow::_CheckGlError(__FILE__, __LINE__);

    std::vector<glow::vec2> scanIndexes;
    glow::GlBuffer<glow::vec2> bufReadBuffer{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STREAM_READ};
    bufReadBuffer.resize(maxPointsPerScan_);

    ScanInfo current;
    int32_t currentScanIndex{-1};

    uint32_t count = 0;
    while (count * maxPointsPerScan_ < bufScanIndexes_.size()) {
      uint32_t read_size = std::min<uint32_t>(maxPointsPerScan_, bufScanIndexes_.size() - count * maxPointsPerScan_);
      bufScanIndexes_.copyTo(count * maxPointsPerScan_, read_size, bufReadBuffer, 0);
      bufReadBuffer.get(scanIndexes, 0, read_size);

      for (uint32_t i = 0; i < read_size; ++i) {
        if (currentScanIndex != int32_t(scanIndexes[i].x)) {
          if (currentScanIndex > -1) scanInfos_[currentScanIndex] = current;
          current.start = count * maxPointsPerScan_ + i;
          current.size = 0;
          currentScanIndex = uint32_t(scanIndexes[i].x);
        }
        current.size += 1;
      }

      ++count;
    }

    std::cout << "copied " << scanInfos_.size() << " scans with " << numCopiedPoints << " points" << std::endl;
  }

  glow::_CheckGlError(__FILE__, __LINE__);

  updateGL();
}

void Viewport::updateHeightmap() {
  // generate height map.
  if (points_.size() == 0) return;

  float groundResolution = 0.5f;
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

  fbMinimumHeightMap_.bind();
  prgMinimumHeightMap_.bind();
  vao_points_.bind();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  prgMinimumHeightMap_.setUniform(GlUniform<float>("minHeight", -3.0f));
  prgMinimumHeightMap_.setUniform(GlUniform<float>("maxHeight", 20.0f));
  prgMinimumHeightMap_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
  prgMinimumHeightMap_.setUniform(GlUniform<float>("tileSize", tileSize_));
  prgMinimumHeightMap_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[0]->pose));

  glDrawArrays(GL_POINTS, 0, bufPoints_.size());

  vao_points_.release();
  prgMinimumHeightMap_.release();
  fbMinimumHeightMap_.release();

  glViewport(vp[0], vp[1], vp[2], vp[3]);
  glDepthFunc(GL_LEQUAL);

  glow::_CheckGlError(__FILE__, __LINE__);
}

void Viewport::updateLabels() {
  glow::_CheckGlError(__FILE__, __LINE__);

  if (labels_.size() == 0) return;

  glow::GlBuffer<uint32_t> bufReadLabels{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STREAM_READ};
  glow::GlBuffer<vec2> bufReadIndexes{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STREAM_READ};
  bufReadLabels.resize(maxPointsPerScan_);
  bufReadIndexes.resize(bufReadLabels.size());

  std::vector<uint32_t> labels(bufReadLabels.size());
  std::vector<vec2> indexes(bufReadIndexes.size());

  uint32_t count = 0;
  uint32_t max_size = bufReadLabels.size();
  uint32_t buffer_size = bufLabels_.size();

  while (count * max_size < bufLabels_.size()) {
    uint32_t size = std::min<uint32_t>(max_size, buffer_size - count * max_size);

    bufLabels_.copyTo(count * max_size, size, bufReadLabels, 0);
    bufScanIndexes_.copyTo(count * max_size, size, bufReadIndexes, 0);

    bufReadLabels.get(labels, 0, size);
    bufReadIndexes.get(indexes, 0, size);

    for (uint32_t i = 0; i < size; ++i) {
      uint32_t scanidx = indexes[i].x;
      uint32_t idx = indexes[i].y;
      (*labels_[scanidx])[idx] = labels[i];
    }

    count++;
  }

  glow::_CheckGlError(__FILE__, __LINE__);
}

void Viewport::setRadius(float value) { mRadius = value; }

void Viewport::setLabel(uint32_t label) { mCurrentLabel = label; }

void Viewport::setLabelColors(const std::map<uint32_t, glow::GlColor>& colors) {
  mLabelColors = colors;

  std::vector<uint8_t> label_colors(3 * 1024, 0);
  for (auto it = mLabelColors.begin(); it != mLabelColors.end(); ++it) {
    if (it->first > 1023) {
      throw std::runtime_error("currently only labels up to 1024 are supported.");
    }
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

  if (removeGround_) updateHeightmap();

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
  // update.

  ScopedBinder<GlVertexArray> vaoBinder(vao_points_);
  ScopedBinder<GlProgram> programBinder(prgUpdateVisibility_);
  ScopedBinder<GlTransformFeedback> feedbackBinder(tfUpdateVisibility_);

  prgUpdateVisibility_.setUniform(GlUniform<uint32_t>("label", label));
  prgUpdateVisibility_.setUniform(GlUniform<uint32_t>("visibility", visible ? 1 : 0));

  glEnable(GL_RASTERIZER_DISCARD);

  uint32_t count = 0;
  uint32_t max_size = bufUpdatedVisiblity_.size();
  while (count * max_size < bufVisible_.size()) {
    uint32_t size = std::min<uint32_t>(max_size, bufVisible_.size() - count * max_size);

    tfUpdateVisibility_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, count * max_size, size);
    tfUpdateVisibility_.end();

    bufUpdatedVisiblity_.copyTo(0, size, bufVisible_, count * max_size);
    ++count;
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

  bool showSingleScan = drawingOption_["single scan"];

  if (points_.size() > 0) {
    glPointSize(pointSize_);

    ScopedBinder<GlProgram> program_binder(prgDrawPoints_);
    ScopedBinder<GlVertexArray> vao_binder(vao_points_);

    prgDrawPoints_.setUniform(GlUniform<bool>("useRemission", drawingOption_["remission"]));
    prgDrawPoints_.setUniform(GlUniform<bool>("useColor", drawingOption_["color"]));
    prgDrawPoints_.setUniform(GlUniform<bool>("removeGround", removeGround_));
    prgDrawPoints_.setUniform(GlUniform<float>("groundThreshold", groundThreshold_));
    prgDrawPoints_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
    prgDrawPoints_.setUniform(GlUniform<float>("tileSize", tileSize_));
    prgDrawPoints_.setUniform(GlUniform<bool>("showAllPoints", drawingOption_["show all points"]));
    prgDrawPoints_.setUniform(GlUniform<int32_t>("heightMap", 1));

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.bind();

    glActiveTexture(GL_TEXTURE1);
    texMinimumHeightMap_.bind();

    mvp_ = projection_ * view_ * conversion_;
    prgDrawPoints_.setUniform(mvp_);

    if (showSingleScan)
      glDrawArrays(GL_POINTS, scanInfos_[singleScanIdx_].start, scanInfos_[singleScanIdx_].size);
    else
      glDrawArrays(GL_POINTS, 0, bufPoints_.size());

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.release();
    glActiveTexture(GL_TEXTURE1);
    texMinimumHeightMap_.release();
  }

  if (showSingleScan) {
    ScopedBinder<GlProgram> program_binder(prgDrawPose_);
    ScopedBinder<GlVertexArray> vao_binder(vao_no_points_);
    mvp_ = projection_ * view_ * conversion_;

    prgDrawPose_.setUniform(mvp_);
    prgDrawPose_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[singleScanIdx_]->pose));
    prgDrawPose_.setUniform(GlUniform<float>("size", 0.5f));

    glDrawArrays(GL_POINTS, 0, 1);
  }

  if (drawingOption_["show camera"]) {
    ScopedBinder<GlProgram> program_binder(prgDrawFrustum_);
    ScopedBinder<GlVertexArray> vao_binder(vao_no_points_);

    prgDrawFrustum_.setUniform(mvp_);
    prgDrawFrustum_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[singleScanIdx_]->pose));
    prgDrawFrustum_.setUniform(GlUniform<int32_t>("width", 1241));
    prgDrawFrustum_.setUniform(GlUniform<int32_t>("height", 376));

    glDrawArrays(GL_POINTS, 0, 1);
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

  if (event->modifiers() == Qt::ControlModifier) {
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
  prgUpdateLabels_.setUniform(GlUniform<bool>("removeGround", removeGround_));
  prgUpdateLabels_.setUniform(GlUniform<float>("groundThreshold", groundThreshold_));
  prgUpdateLabels_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
  prgUpdateLabels_.setUniform(GlUniform<float>("tileSize", tileSize_));
  prgUpdateLabels_.setUniform(GlUniform<bool>("showAllPoints", drawingOption_["show all points"]));
  prgUpdateLabels_.setUniform(GlUniform<int32_t>("heightMap", 1));
  mvp_ = projection_ * mCamera.matrix() * conversion_;
  prgUpdateLabels_.setUniform(mvp_);

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

  uint32_t count = 0;
  uint32_t max_size = bufUpdatedLabels_.size();
  uint32_t buffer_start = 0;
  uint32_t buffer_size = bufLabels_.size();

  if (showSingleScan) {
    buffer_start = scanInfos_[singleScanIdx_].start;
    buffer_size = scanInfos_[singleScanIdx_].size;
  }

  while (count * max_size < buffer_size) {
    uint32_t size = std::min<uint32_t>(max_size, buffer_size - count * max_size);

    tfUpdateLabels_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, buffer_start + count * max_size, size);
    tfUpdateLabels_.end();

    bufUpdatedLabels_.copyTo(0, size, bufLabels_, buffer_start + count * max_size);

    count++;
  }

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

void Viewport::centerOnCurrentTile() {
  // have to convert from robotic coordinate system to the opengl system.
  if (points_.size() == 0) return;

  Eigen::Vector4f t = points_[0]->pose.col(3);

  mCamera.lookAt(-tilePos_.y + 20, t.z() + 25, -tilePos_.x + 20, -tilePos_.y, t.z(), -tilePos_.x);
  updateGL();
}
