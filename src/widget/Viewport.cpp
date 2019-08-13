#include "Viewport.h"
#include "data/Math.h"
#include "data/draw_utils.h"
#include "data/misc.h"

#include <glow/GlCapabilities.h>
#include <glow/ScopedBinder.h>
#include <glow/glutil.h>
#include <QtWidgets/QMessageBox>
#include <algorithm>
#include <chrono>
#include <fstream>
#include "rv/Stopwatch.h"

#include <glow/GlState.h>
#include <deque>

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
      texTempHeightMap_(100, 100, TextureFormat::R_FLOAT),
      texTriangles_(3 * 100, 1, TextureFormat::RGB),
      fboOffscreen_(100, 100),
      texOffscreen_(100, 100, TextureFormat::RG_FLOAT) {
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

  bufSelectedLabels_.resize(maxPointsPerScan_);
  tfSelectedLabels_.attach({"out_label"}, bufSelectedLabels_);

  initPrograms();
  initVertexBuffers();

  drawingOption_["remission"] = true;
  drawingOption_["color"] = false;
  drawingOption_["single scan"] = false;
  drawingOption_["show all points"] = false;
  drawingOption_["draw heightmap"] = false;
  drawingOption_["draw triangles"] = false;
  drawingOption_["show plane"] = true;
  drawingOption_["draw instances"] = false;
  drawingOption_["add car points"] = false;

  texLabelColors_.setMinifyingOperation(TexRectMinOp::NEAREST);
  texLabelColors_.setMagnifyingOperation(TexRectMagOp::NEAREST);

  texTriangles_.setMinifyingOperation(TexRectMinOp::NEAREST);
  texTriangles_.setMagnifyingOperation(TexRectMagOp::NEAREST);

  texMinimumHeightMap_.setMinifyingOperation(TexMinOp::NEAREST);
  texMinimumHeightMap_.setMagnifyingOperation(TexMagOp::NEAREST);

  texTempHeightMap_.setMinifyingOperation(TexMinOp::NEAREST);
  texTempHeightMap_.setMagnifyingOperation(TexMagOp::NEAREST);

  fbMinimumHeightMap_.attach(FramebufferAttachment::COLOR0, texMinimumHeightMap_);
  GlRenderbuffer depthbuffer(fbMinimumHeightMap_.width(), fbMinimumHeightMap_.height(),
                             RenderbufferFormat::DEPTH_STENCIL);
  fbMinimumHeightMap_.attach(FramebufferAttachment::DEPTH_STENCIL, depthbuffer);

  setAutoFillBackground(false);

  cameras_["Default"] = std::make_shared<RoSeCamera>();
  cameras_["CAD"] = std::make_shared<CADCamera>();
  mCamera = cameras_["Default"];

  nnSampler_.setMinifyingOperation(TexMinOp::NEAREST);
  nnSampler_.setMagnifyingOperation(TexMagOp::NEAREST);

  texOffscreen_.setMagnifyingOperation(TexMagOp::NEAREST);
  texOffscreen_.setMinifyingOperation(TexMinOp::NEAREST);

  glow::_CheckGlError(__FILE__, __LINE__);
}

Viewport::~Viewport() {
  // workaround for strange behaviour with my Nvidia 860m. Even though the buffers fit into memory, they mess up
  // something up. However, I noticed that it does not cause havoc if the buffers are empty or small.

  //  bufPoints_.assign(std::vector<vec4>());
  //  bufLabels_.assign(std::vector<uint32_t>());
  //  bufScanIndexes_.assign(std::vector<vec2>());
  //  bufVisible_.assign(std::vector<uint32_t>());

  texTempHeightMap_.resize(1, 1);
  texMinimumHeightMap_.resize(1, 1);

  std::vector<PointcloudPtr> points;
  std::vector<LabelsPtr> labels;
  setPoints(points, labels);
  setPoints(points, labels);
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

  prgDrawHeightmap_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/draw_heightmap.vert"));
  prgDrawHeightmap_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/draw_heightmap.geom"));
  prgDrawHeightmap_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawHeightmap_.link();

  prgAverageHeightMap_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/empty.vert"));
  prgAverageHeightMap_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/quad.geom"));
  prgAverageHeightMap_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/average_heightmap.frag"));
  prgAverageHeightMap_.link();

  prgDrawPlane_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/empty.vert"));
  prgDrawPlane_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/draw_plane.geom"));
  prgDrawPlane_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawPlane_.link();

  prgDrawBoundingBoxes_.attach(glow::GlShader::fromCache(glow::ShaderType::VERTEX_SHADER, "shaders/draw_bbox.vert"));
  prgDrawBoundingBoxes_.attach(glow::GlShader::fromCache(glow::ShaderType::GEOMETRY_SHADER, "shaders/draw_bbox.geom"));
  prgDrawBoundingBoxes_.attach(
      glow::GlShader::fromCache(glow::ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawBoundingBoxes_.link();

  prgDrawBoundingBoxesId_.attach(glow::GlShader::fromCache(glow::ShaderType::VERTEX_SHADER, "shaders/draw_bbox.vert"));
  prgDrawBoundingBoxesId_.attach(
      glow::GlShader::fromCache(glow::ShaderType::GEOMETRY_SHADER, "shaders/draw_bbox_id.geom"));
  prgDrawBoundingBoxesId_.attach(
      glow::GlShader::fromCache(glow::ShaderType::FRAGMENT_SHADER, "shaders/draw_bbox_id.frag"));
  prgDrawBoundingBoxesId_.link();

  prgDrawSelectedBoundingBox_.attach(
      glow::GlShader::fromCache(glow::ShaderType::VERTEX_SHADER, "shaders/draw_bbox.vert"));
  prgDrawSelectedBoundingBox_.attach(
      glow::GlShader::fromCache(glow::ShaderType::GEOMETRY_SHADER, "shaders/draw_bbox_id.geom"));
  prgDrawSelectedBoundingBox_.attach(
      glow::GlShader::fromCache(glow::ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawSelectedBoundingBox_.link();

  prgGetSelectedLabels_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/selected_labels.vert"));
  prgGetSelectedLabels_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/selected_labels.geom"));
  prgGetSelectedLabels_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/empty.frag"));
  prgGetSelectedLabels_.attach(tfSelectedLabels_);
  prgGetSelectedLabels_.link();

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

  vao_heightmap_points_.setVertexAttribute(0, bufHeightMapPoints_, 2, AttributeType::FLOAT, false, sizeof(glow::vec2),
                                           nullptr);

  vao_bboxes_.setVertexAttribute(0, bufBboxPositionsYaw_, 4, glow::AttributeType::FLOAT, false, 4 * sizeof(float),
                                 nullptr);
  vao_bboxes_.setVertexAttribute(1, bufBboxSizeIds_, 4, glow::AttributeType::FLOAT, false, 4 * sizeof(float), nullptr);

  vao_bboxes_.setVertexAttribute(2, bufBboxInstanceLabels_, 1, glow::AttributeType::UNSIGNED_INT, false,
                                 1 * sizeof(uint32_t), nullptr);
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
    prgFillTilePoints_.setUniform(GlUniform<bool>("addCarPoints", drawingOption_["add car points"]));

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

    // write last entry.
    if (currentScanIndex > -1) scanInfos_[currentScanIndex] = current;

    // ensure that empty indexes get the beginning from before.
    for (uint32_t i = 0; i < scanInfos_.size(); ++i) {
      if (scanInfos_[i].size == 0 && i > 0) scanInfos_[i].start = scanInfos_[i - 1].start + scanInfos_[i - 1].size;
    }

    std::cout << "copied " << scanInfos_.size() << " scans with " << numCopiedPoints << " points" << std::endl;
    if (numCopiedPoints == bufPoints_.capacity()) {
      QMessageBox::warning(this, "Increase number of scans.",
                           "Its possible that not all scans could be loaded. Please increase the 'max scans' in "
                           "the settings.cfg, but ensure that enough GPU memory is available.");
    }
  }

  glow::_CheckGlError(__FILE__, __LINE__);

  updateLabels();

  if (labelInstances_) {
    updateBoundingBoxes();

    fillBoundingBoxBuffers();
    updateInstanceSelectionMap();  // re-render selection map.
  }

  updateGL();
}

void Viewport::updateHeightmap() {
  // generate height map.
  if (points_.size() == 0) return;

  uint32_t width = tileSize_ / groundResolution_;
  uint32_t height = tileSize_ / groundResolution_;

  if (fbMinimumHeightMap_.width() != width || fbMinimumHeightMap_.height() != height) {
    fbMinimumHeightMap_.resize(width, height);
    texMinimumHeightMap_.resize(width, height);
    texTempHeightMap_.resize(width, height);

    // update also depth buffer.
    GlRenderbuffer depthbuffer(fbMinimumHeightMap_.width(), fbMinimumHeightMap_.height(),
                               RenderbufferFormat::DEPTH_STENCIL);
    fbMinimumHeightMap_.attach(FramebufferAttachment::COLOR0, texTempHeightMap_);
    fbMinimumHeightMap_.attach(FramebufferAttachment::DEPTH_STENCIL, depthbuffer);

    std::vector<glow::vec2> indexes(width * height);
    for (uint32_t x = 0; x < width; ++x) {
      for (uint32_t y = 0; y < height; ++y) {
        indexes[x + y * width] = vec2(float(x + 0.5f) / width, float(y + 0.5f) / height);
      }
    }

    //  std::cout << "w x h: " << width << " x " << height << std::endl;

    //  std::cout << indexes[0] << ", " << indexes[10] << std::endl;
    bufHeightMapPoints_.assign(indexes);
  }

  fbMinimumHeightMap_.attach(FramebufferAttachment::COLOR0, texTempHeightMap_);

  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);
  glPointSize(1.0f);

  glViewport(0, 0, fbMinimumHeightMap_.width(), fbMinimumHeightMap_.height());

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  fbMinimumHeightMap_.bind();
  prgMinimumHeightMap_.bind();
  vao_points_.bind();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  prgMinimumHeightMap_.setUniform(GlUniform<float>("minHeight", -3.0f));
  prgMinimumHeightMap_.setUniform(GlUniform<float>("maxHeight", 5.0f));
  prgMinimumHeightMap_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
  prgMinimumHeightMap_.setUniform(GlUniform<float>("tileSize", tileSize_));
  prgMinimumHeightMap_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[0]->pose));

  glDrawArrays(GL_POINTS, 0, bufPoints_.size());

  prgMinimumHeightMap_.release();
  vao_points_.release();

  fbMinimumHeightMap_.attach(FramebufferAttachment::COLOR0, texMinimumHeightMap_);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glDisable(GL_DEPTH_TEST);

  vao_no_points_.bind();
  prgAverageHeightMap_.bind();
  glActiveTexture(GL_TEXTURE0);
  texTempHeightMap_.bind();

  glDrawArrays(GL_POINTS, 0, 1);

  texTempHeightMap_.release();
  prgAverageHeightMap_.release();
  vao_no_points_.release();
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

  labeledCount_ = 0;

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
      labeledCount_ += (labels[i] != 0);
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

  // trigger update.
  if (name == "show all moving instances") {
    fillBoundingBoxBuffers();
    updateInstanceSelectionMap();
  }

  if (name == "follow pose") {
    //    std::cout << "before \n" << camera_.matrix() << std::endl;
    if (value == false)  // currently following, but now defollowing:
    {
      mCamera->setMatrix(mCamera->matrix() * conversion_ * points_[singleScanIdx_]->pose.inverse() *
                         conversion_.inverse());
    } else {
      mCamera->setMatrix(mCamera->matrix() * conversion_ * points_[singleScanIdx_]->pose * conversion_.inverse());
    }
    //    std::cout << "after \n" << camera_.matrix() << std::endl;
  }

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
  // bounding boxes of moving class must be updated
  if (labelInstances_ || drawingOption_["draw instance boxes"]) {
    fillBoundingBoxBuffers();
    updateInstanceSelectionMap();  // re-render selection map.
  }

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

  mCamera->lookAt(5.0f, 5.0f, 5.0f, 0.0f, 0.0f, 0.0f);
}

void Viewport::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);

  float aspect = float(w) / float(h);

  if (projectionMode_ == CameraProjection::perspective) {
    float fov = radians(45.0f);
    projection_ = glPerspective(fov, aspect, 0.1f, 2000.0f);
  } else {
    float fov = 10.0f;
    if (w <= h)
      projection_ = glOrthographic(-fov, fov, -fov / aspect, fov / aspect, 0.1f, 2000.0f);
    else
      projection_ = glOrthographic(-fov * aspect, fov * aspect, -fov, fov, 0.1, 2000.0f);
  }

  // update selection rendering.
  fboOffscreen_.resize(width(), height());
  texOffscreen_.resize(width(), height());

  fboOffscreen_.attach(FramebufferAttachment::COLOR0, texOffscreen_);
  GlRenderbuffer rb(width(), height(), RenderbufferFormat::DEPTH_STENCIL);
  fboOffscreen_.attach(FramebufferAttachment::DEPTH_STENCIL, rb);

  updateInstanceSelectionMap();
}

void Viewport::paintGL() {
  glow::_CheckGlError(__FILE__, __LINE__);

  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPointSize(pointSize_);

  view_ = mCamera->matrix();

  if (drawingOption_["follow pose"]) {
    //    view_ = view_ * conversion_ * currentFrame_->pose.inverse() * conversion_.inverse();

    RoSeCamera dummy;
    dummy.setMatrix(conversion_ * points_[singleScanIdx_]->pose.inverse() * conversion_.inverse());
    view_ = view_ * dummy.matrix();
  }

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
  bool showScanRange = drawingOption_["show scan range"];

  prgDrawPoints_.setUniform(GlUniform<int32_t>("colormap", remissionColormap_));
  prgDrawPoints_.setUniform(GlUniform<float>("gamma", gammaCorrection_));

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
    prgDrawPoints_.setUniform(GlUniform<bool>("planeRemovalNormal", planeRemovalNormal_));
    prgDrawPoints_.setUniform(GlUniform<Eigen::Vector3f>("planeNormal", planeNormal_));
    prgDrawPoints_.setUniform(GlUniform<float>("planeThresholdNormal", planeThresholdNormal_));
    prgDrawPoints_.setUniform(GlUniform<float>("planeDirectionNormal", planeDirectionNormal_));
    prgDrawPoints_.setUniform(GlUniform<bool>("drawInstances", false));

    prgDrawPoints_.setUniform(GlUniform<bool>("hideLabeledInstances", drawingOption_["hide labeled instances"]));

    //    prgDrawPoints_.setUniform(GlUniform<bool>("carAsBase", drawingOption_["carAsBase"]));
    Eigen::Matrix4f plane_pose = Eigen::Matrix4f::Identity();
    plane_pose(0, 3) = tilePos_.x;
    plane_pose(1, 3) = tilePos_.y;
    plane_pose(2, 3) = points_[0]->pose(2, 3);
    if (drawingOption_["carAsBase"] && points_.size() > singleScanIdx_) {
      plane_pose = points_[singleScanIdx_]->pose;
      //      plane_pose.col(3) = points_[singleScanIdx_]->pose.col(3);
    }

    prgDrawPoints_.setUniform(GlUniform<Eigen::Matrix4f>("plane_pose", plane_pose));

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.bind();

    glActiveTexture(GL_TEXTURE1);
    texMinimumHeightMap_.bind();

    prgDrawPoints_.setUniform(mvp_);

    if (showSingleScan)
      glDrawArrays(GL_POINTS, scanInfos_[singleScanIdx_].start, scanInfos_[singleScanIdx_].size);
    else if (showScanRange) {
      uint32_t start = scanInfos_[scanRangeBegin_].start;
      uint32_t count =
          scanInfos_[scanRangeEnd_].start + scanInfos_[scanRangeEnd_].size - scanInfos_[scanRangeBegin_].start;
      glDrawArrays(GL_POINTS, start, count);
    } else
      glDrawArrays(GL_POINTS, 0, bufPoints_.size());

    if (drawingOption_["draw instances"]) {
      // draw points of specific instance larger.
      glPointSize(pointSize_ + 2);

      prgDrawPoints_.setUniform(GlUniform<bool>("drawInstances", true));

      prgDrawPoints_.setUniform(GlUniform<uint32_t>("selectedInstanceId", selectedInstanceId_));
      prgDrawPoints_.setUniform(GlUniform<uint32_t>("selectedInstanceLabel", selectedInstanceLabel_));

      if (showSingleScan)
        glDrawArrays(GL_POINTS, scanInfos_[singleScanIdx_].start, scanInfos_[singleScanIdx_].size);
      else if (showScanRange) {
        uint32_t start = scanInfos_[scanRangeBegin_].start;
        uint32_t count =
            scanInfos_[scanRangeEnd_].start + scanInfos_[scanRangeEnd_].size - scanInfos_[scanRangeBegin_].start;
        glDrawArrays(GL_POINTS, start, count);

      } else
        glDrawArrays(GL_POINTS, 0, bufPoints_.size());

      glPointSize(pointSize_);
    }

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.release();
    glActiveTexture(GL_TEXTURE1);
    texMinimumHeightMap_.release();
  }

  if ((labelInstances_ || drawingOption_["draw instance boxes"]) && bufBboxPositionsYaw_.size() > 0) {
    prgDrawBoundingBoxes_.bind();
    prgDrawBoundingBoxes_.setUniform(glow::GlUniform<Eigen::Matrix4f>("mvp", mvp_));
    prgDrawBoundingBoxes_.setUniform(GlUniform<uint32_t>("selectedInstanceId", selectedInstanceId_));
    prgDrawBoundingBoxes_.setUniform(GlUniform<uint32_t>("selectedInstanceLabel", selectedInstanceLabel_));
    //    prgDrawBoundingBoxes_.setUniform(glow::GlUniform<glow::vec3>("in_color", glow::vec3(1, 0, 0)));
    glLineWidth(1.0f);

    vao_bboxes_.bind();
    glDrawArrays(GL_POINTS, 0, bufBboxPositionsYaw_.size());

    prgDrawBoundingBoxes_.release();

    uint32_t instance_label = (selectedInstanceId_ << 16) | selectedInstanceLabel_;
    if (id2idx_.find(instance_label) != id2idx_.end()) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // FIXME: state changed.
      prgDrawSelectedBoundingBox_.bind();
      prgDrawSelectedBoundingBox_.setUniform(glow::GlUniform<Eigen::Matrix4f>("mvp", mvp_));
      prgDrawSelectedBoundingBox_.setUniform(GlUniform<bool>("use_custom_color", true));

      GlColor selectionColor = GlColor::GOLD;
      selectionColor.A = 0.5f;
      prgDrawSelectedBoundingBox_.setUniform(GlUniform<GlColor>("in_color", selectionColor));

      glDrawArrays(GL_POINTS, id2idx_[instance_label], 1);
      prgDrawSelectedBoundingBox_.release();
      glDisable(GL_BLEND);
    }

    vao_bboxes_.release();

    glLineWidth(1.0f);

    prgDrawBoundingBoxes_.release();
  }

  if (planeRemovalNormal_ && drawingOption_["show plane"]) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // FIXME: state changed.

    ScopedBinder<GlProgram> program_binder(prgDrawPlane_);
    ScopedBinder<GlVertexArray> vao_binder(vao_no_points_);

    prgDrawPlane_.setUniform(mvp_);

    prgDrawPlane_.setUniform(GlUniform<bool>("planeRemovalNormal", planeRemovalNormal_));
    prgDrawPlane_.setUniform(GlUniform<float>("planeThresholdNormal", planeThresholdNormal_));
    prgDrawPlane_.setUniform(GlUniform<float>("planeDirectionNormal", planeDirectionNormal_));
    prgDrawPlane_.setUniform(GlUniform<Eigen::Vector3f>("planeNormal", planeNormal_));
    //    prgDrawPoints_.setUniform(GlUniform<bool>("carAsBase", drawingOption_["carAsBase"]));
    Eigen::Matrix4f plane_pose = Eigen::Matrix4f::Identity();

    plane_pose(0, 3) = tilePos_.x;
    plane_pose(1, 3) = tilePos_.y;
    if (points_.size() > 0) plane_pose(2, 3) = points_[0]->pose(2, 3);
    if (drawingOption_["carAsBase"] && points_.size() > singleScanIdx_) {
      plane_pose = points_[singleScanIdx_]->pose;
      //      plane_pose.col(3) = points_[singleScanIdx_]->pose.col(3);
    }

    prgDrawPlane_.setUniform(GlUniform<Eigen::Matrix4f>("plane_pose", plane_pose));
    glDrawArrays(GL_POINTS, 0, 1);

    glDisable(GL_BLEND);
  }

  if (showSingleScan) {
    ScopedBinder<GlProgram> program_binder(prgDrawPose_);
    ScopedBinder<GlVertexArray> vao_binder(vao_no_points_);

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

  if (drawingOption_["draw heightmap"]) {
    ScopedBinder<GlProgram> program_binder(prgDrawHeightmap_);
    ScopedBinder<GlVertexArray> vao_binder(vao_heightmap_points_);
    glActiveTexture(GL_TEXTURE0);
    texMinimumHeightMap_.bind();

    prgDrawHeightmap_.setUniform(mvp_);
    prgDrawHeightmap_.setUniform(GlUniform<float>("ground_resolution", groundResolution_));
    prgDrawHeightmap_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
    prgDrawHeightmap_.setUniform(GlUniform<float>("tileSize", tileSize_));

    glDrawArrays(GL_POINTS, 0, bufHeightMapPoints_.size());

    texMinimumHeightMap_.release();
  }

  glDisable(GL_DEPTH_TEST);
  // Important: QPainter is apparently not working with OpenGL Core Profile < Qt5.9!!!!
  //  http://blog.qt.io/blog/2017/01/27/opengl-core-profile-context-support-qpainter/
  //  QPainter painter(this); // << does not work with OpenGL Core Profile.
  if (mMode == POLYGON || (labelInstances_ && instanceSelected_) || instanceLabelingMode_ == 3) {
    ScopedBinder<GlProgram> program_binder(prgPolygonPoints_);

    vao_polygon_points_.bind();

    uint32_t label = mCurrentLabel;
    if (labelInstances_ && instanceSelected_) label = selectedInstanceLabel_;
    if (instanceLabelingMode_ == 3) label = 0;

    prgPolygonPoints_.setUniform(GlUniform<int32_t>("width", width()));
    prgPolygonPoints_.setUniform(GlUniform<int32_t>("height", height()));
    prgPolygonPoints_.setUniform(GlUniform<uint32_t>("label", label));

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.bind();

    glDrawArrays(GL_LINE_LOOP, 0, bufPolygonPoints_.size());

    glPointSize(7.0f);

    glDrawArrays(GL_POINTS, 0, bufPolygonPoints_.size());
    vao_polygon_points_.release();

    if (drawingOption_["draw triangles"]) {
      vao_triangles_.bind();

      glDrawArrays(GL_LINES, 0, bufTriangles_.size());

      vao_triangles_.release();
    }
    texLabelColors_.release();
  }

  glow::_CheckGlError(__FILE__, __LINE__);
}

void Viewport::updateInstanceSelectionMap() {
  view_ = mCamera->matrix();

  if (drawingOption_["follow pose"]) {
    //    view_ = view_ * conversion_ * currentFrame_->pose.inverse() * conversion_.inverse();

    RoSeCamera dummy;
    dummy.setMatrix(conversion_ * points_[singleScanIdx_]->pose.inverse() * conversion_.inverse());
    view_ = view_ * dummy.matrix();
  }

  mvp_ = projection_ * view_ * conversion_;

  // store viewport, and clear color:
  GLfloat cc[4];
  GLint vp[4];

  glGetIntegerv(GL_VIEWPORT, vp);
  glGetFloatv(GL_COLOR_CLEAR_VALUE, cc);
  glDisable(GL_LINE_SMOOTH);
  //  glDisable(GL_POINT_SMOOTH);
  //  glDisable(GL_MULTISAMPLE);
  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);

  fboOffscreen_.bind();
  prgDrawBoundingBoxesId_.bind();
  glClearColor(0, 0, 0, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  prgDrawBoundingBoxesId_.setUniform(glow::GlUniform<Eigen::Matrix4f>("mvp", mvp_));
  //    prgDrawBoundingBoxes_.setUniform(glow::GlUniform<glow::vec3>("in_color", glow::vec3(1, 0, 0)));

  vao_bboxes_.bind();
  glDrawArrays(GL_POINTS, 0, bufBboxPositionsYaw_.size());
  vao_bboxes_.release();

  prgDrawBoundingBoxesId_.release();
  fboOffscreen_.release();
  glViewport(vp[0], vp[1], vp[2], vp[3]);
  glClearColor(cc[0], cc[1], cc[2], cc[3]);
  CheckGlError();

  texOffscreen_.download(offscreenContent_);

  glEnable(GL_LINE_SMOOTH);
  //    glDisable(GL_POINT_SMOOTH);
  //    glDisable(GL_MULTISAMPLE);
  //    glDisable(GL_BLEND);
}

uint32_t Viewport::getClickedInstanceId(float x, float y) {
  if (x < 0 || y < 0 || x >= texOffscreen_.width() || y >= texOffscreen_.height()) return 0;

  uint32_t instance_id = offscreenContent_[2 * (x + (texOffscreen_.height() - y) * texOffscreen_.width())];
  uint32_t label_id = offscreenContent_[2 * (x + (texOffscreen_.height() - y) * texOffscreen_.width()) + 1];

  //  std::cout << instance_id << ", " << label_id << " selected" << std::endl;

  return (instance_id << 16 | label_id);
}

std::ostream& operator<<(std::ostream& os, const vec2& v) {
  os << "(" << v.x << ", " << v.y << ")";
  return os;
}

void Viewport::abortPolygonSelection() {
  polygonPoints_.clear();  // start over again.#
  bufPolygonPoints_.assign(polygonPoints_);
  bufTriangles_.resize(0);
  numTriangles_ = 0;
}

void Viewport::wheelEvent(QWheelEvent* event) {
  mChangeCamera = false;

  if (event->modifiers() == Qt::ControlModifier || mMode == PAINT || polygonPoints_.empty()) {
    QPoint numPixels = event->pixelDelta();
    QPoint numDegrees = event->angleDelta() / 8.;
    float delta = 0.0f;

    if (!numPixels.isNull()) {
      delta = numPixels.y();
    } else if (!numDegrees.isNull()) {
      delta = numDegrees.y() / 15.;
    }

    mCamera->wheelEvent(delta, resolveKeyboardModifier(event->modifiers()));
    mChangeCamera = true;

    abortPolygonSelection();
  }
  this->updateGL();
  return;
}

void Viewport::mousePressEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  mChangeCamera = false;

  if (event->modifiers() == Qt::ControlModifier) {
    if (mCamera->mousePressed(event->windowPos().x(), event->windowPos().y(), resolveMouseButtonFlip(event->buttons()),
                              resolveKeyboardModifier(event->modifiers()))) {
      if (pressedkeys.empty()) {
        timer_.start(1 / 60);
      }
      pressedkeys.insert(Qt::Key_F25);  // abuse F25 for mouse events

      mChangeCamera = true;
      abortPolygonSelection();
      updateGL();

      return;
    }
  }

  if (labelInstances_) {
    if ((instanceSelected_ && instanceLabelingMode_ != 4) || (instanceLabelingMode_ == 3)) {
      if (event->buttons() & Qt::LeftButton) {
        if (polygonPoints_.size() == 100) {
          polygonPoints_.back().x = event->x();
          polygonPoints_.back().y = event->y();
        } else {
          polygonPoints_.push_back(vec2(event->x(), event->y()));
        }

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
          std::vector<glow::vec2> tris_verts;

          triangulate(points, triangles);

          //        std::cout << "#triangles: " << triangles.size() << std::endl;

          std::vector<vec3> texContent(3 * 100);
          for (uint32_t i = 0; i < triangles.size(); ++i) {
            auto t = triangles[i];
            texContent[3 * i + 0] = vec3(t.i.x / width(), (height() - t.i.y) / height(), 0);
            texContent[3 * i + 1] = vec3(t.j.x / width(), (height() - t.j.y) / height(), 0);
            texContent[3 * i + 2] = vec3(t.k.x / width(), (height() - t.k.y) / height(), 0);

            tris_verts.push_back(vec2(t.i.x, height() - t.i.y));
            tris_verts.push_back(vec2(t.j.x, height() - t.j.y));
            tris_verts.push_back(vec2(t.j.x, height() - t.j.y));
            tris_verts.push_back(vec2(t.k.x, height() - t.k.y));
            tris_verts.push_back(vec2(t.k.x, height() - t.k.y));
            tris_verts.push_back(vec2(t.i.x, height() - t.i.y));
          }

          numTriangles_ = triangles.size();
          // note: colors are in range [0,1] for FLOAT!
          texTriangles_.assign(PixelFormat::RGB, PixelType::FLOAT, &texContent[0]);
          bufTriangles_.assign(tris_verts);

          if (instanceLabelingMode_ == 3)  // create instance.
          {
            std::vector<uint32_t> selected = getSelectedLabels();
            //            std::cout << selected.size() << " points selected." << std::endl;

            std::map<uint32_t, uint32_t> counts;
            for (auto label : instanceableLabels_) counts[label] = 0;

            for (auto label : selected) {
              if (counts.find(label) != counts.end()) counts[label] += 1;
            }

            // we first have to determine the "instanceable" label inside the selection.
            uint32_t maxLabel = 0;
            uint32_t maxCount = 0;
            for (auto it = counts.begin(); it != counts.end(); ++it) {
              //              std::cout << it->first << ": " << it->second << std::endl;
              if (it->second > maxCount) {
                maxLabel = it->first;
                maxCount = it->second;
              }
            }

            uint32_t selectedInstanceId = 0;
            if (maxLabel > 0) {
              selectedInstanceLabel_ = maxLabel;

              labelPoints(event->x(), event->y(), 0, mCurrentLabel, false);

              updateBoundingBoxes();
              fillBoundingBoxBuffers();
              updateInstanceSelectionMap();

              instanceSelected_ = true;
              selectedInstanceId_ = maxInstanceIds_[selectedInstanceLabel_];
              selectedInstanceId = selectedInstanceId_;
            }

            emit instanceSelected(selectedInstanceId);
          } else {
            labelPoints(event->x(), event->y(), 0, mCurrentLabel, false);

            updateBoundingBoxes();
            fillBoundingBoxBuffers();
            updateInstanceSelectionMap();
          }
        }

        polygonPoints_.clear();
      }

      bufPolygonPoints_.assign(polygonPoints_);

      repaint();
    }
  } else {
    if (mMode == PAINT) {
      buttonPressed = true;
      mChangeCamera = false;
      if (event->buttons() & Qt::LeftButton)
        labelPoints(event->x(), event->y(), mRadius, mCurrentLabel, false);
      else if (event->buttons() & Qt::RightButton)
        labelPoints(event->x(), event->y(), mRadius, mCurrentLabel, true);

      updateGL();
    } else if (mMode == POLYGON) {
      if (event->buttons() & Qt::LeftButton) {
        if (polygonPoints_.size() == 100) {
          polygonPoints_.back().x = event->x();
          polygonPoints_.back().y = event->y();
        } else {
          polygonPoints_.push_back(vec2(event->x(), event->y()));
        }

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
          std::vector<glow::vec2> tris_verts;

          triangulate(points, triangles);

          //        std::cout << "#triangles: " << triangles.size() << std::endl;

          std::vector<vec3> texContent(3 * 100);
          for (uint32_t i = 0; i < triangles.size(); ++i) {
            auto t = triangles[i];
            texContent[3 * i + 0] = vec3(t.i.x / width(), (height() - t.i.y) / height(), 0);
            texContent[3 * i + 1] = vec3(t.j.x / width(), (height() - t.j.y) / height(), 0);
            texContent[3 * i + 2] = vec3(t.k.x / width(), (height() - t.k.y) / height(), 0);

            tris_verts.push_back(vec2(t.i.x, height() - t.i.y));
            tris_verts.push_back(vec2(t.j.x, height() - t.j.y));
            tris_verts.push_back(vec2(t.j.x, height() - t.j.y));
            tris_verts.push_back(vec2(t.k.x, height() - t.k.y));
            tris_verts.push_back(vec2(t.k.x, height() - t.k.y));
            tris_verts.push_back(vec2(t.i.x, height() - t.i.y));
          }

          numTriangles_ = triangles.size();
          // note: colors are in range [0,1] for FLOAT!
          texTriangles_.assign(PixelFormat::RGB, PixelType::FLOAT, &texContent[0]);
          bufTriangles_.assign(tris_verts);

          labelPoints(event->x(), event->y(), 0, mCurrentLabel, false);
        }

        polygonPoints_.clear();
      }

      bufPolygonPoints_.assign(polygonPoints_);

      repaint();
    }
  }

  event->accept();
}

void Viewport::mouseReleaseEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (mChangeCamera) {
    pressedkeys.erase(Qt::Key_F25);  // abuse F25 for mouse events
    if (pressedkeys.empty()) {
      timer_.stop();
    }
    updateGL();
    if (mCamera->mouseReleased(event->windowPos().x(), event->windowPos().y(), resolveMouseButtonFlip(event->buttons()),
                               resolveKeyboardModifier(event->modifiers()))) {
      // timer_.stop();
      updateGL();  // get the last action.

      updateInstanceSelectionMap();

      return;
    }
  } else {
    if (labelInstances_) {
      if (instanceSelectionMode_) {
        std::cout << "instance selection mode." << std::endl;
        uint32_t instance_label = getClickedInstanceId(event->x(), event->y());

        if (instanceLabelingMode_ == 4) {
          std::cout << "join. " << std::endl;
          if (instanceSelected_) {
            std::cout << "first already selected." << std::endl;
            instanceSelectionMode_ = false;

            uint32_t selectedInstanceId_before = selectedInstanceId_;
            uint32_t selectedInstanceLabel_before = selectedInstanceLabel_;

            selectedInstanceId_ = (instance_label >> 16);
            selectedInstanceLabel_ = (instance_label & uint32_t(0xFFFF));

            if (selectedInstanceLabel_ == selectedInstanceLabel_before) {
              // trigger relabeling.
              std::cout << "relabeling." << std::endl;
              labelPoints(event->x(), event->y(), 0, selectedInstanceId_before, false);

              updateBoundingBoxes();
              fillBoundingBoxBuffers();
              updateInstanceSelectionMap();
            }

            // select new joined instances.
            selectedInstanceId_ = selectedInstanceId_before;
            selectedInstanceLabel_ = selectedInstanceLabel_before;

          } else {
            if (instance_label > 0) {
              instanceSelected_ = true;
              selectedInstanceId_ = (instance_label >> 16);
              selectedInstanceLabel_ = (instance_label & uint32_t(0xFFFF));

            } else {
              instanceSelectionMode_ = false;
              selectedInstanceId_ = 0;
              selectedInstanceLabel_ = 0;
            }
          }
        } else {
          instanceSelectionMode_ = false;

          instanceSelected_ = false;
          selectedInstanceId_ = 0;
          selectedInstanceLabel_ = 0;

          if (instance_label > 0) {
            instanceSelected_ = true;
            selectedInstanceId_ = (instance_label >> 16);
            selectedInstanceLabel_ = (instance_label & uint32_t(0xFFFF));
          }
        }

        emit instanceSelected(instance_label);

        updateGL();
      } else {
        if (instanceSelected_ || (instanceLabelingMode_ == 3)) {
          if (polygonPoints_.size() > 0) {
            polygonPoints_.back().x = event->x();
            polygonPoints_.back().y = event->y();

            bufPolygonPoints_.assign(polygonPoints_);
          }

          repaint();
        }
      }
    } else {
      if (mMode == PAINT) {
        buttonPressed = false;

        if (event->buttons() & Qt::LeftButton)
          labelPoints(event->x(), event->y(), mRadius, mCurrentLabel, false);
        else if (event->buttons() & Qt::RightButton)
          labelPoints(event->x(), event->y(), mRadius, mCurrentLabel, true);

        updateGL();
      } else if (mMode == POLYGON) {
        if (polygonPoints_.size() > 0) {
          polygonPoints_.back().x = event->x();
          polygonPoints_.back().y = event->y();

          bufPolygonPoints_.assign(polygonPoints_);
        }

        repaint();
      }
    }
  }

  event->accept();
}

void Viewport::mouseMoveEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (mChangeCamera) {
    if (mCamera->mouseMoved(event->windowPos().x(), event->windowPos().y(), resolveMouseButtonFlip(event->buttons()),
                            resolveKeyboardModifier(event->modifiers()))) {
      return;
    }
  }

  if (labelInstances_) {
    if ((instanceSelected_ && !instanceSelectionMode_) || (instanceLabelingMode_ == 3)) {
      if (polygonPoints_.size() > 0) {
        polygonPoints_.back().x = event->x();
        polygonPoints_.back().y = event->y();

        bufPolygonPoints_.assign(polygonPoints_);

        repaint();
      }
    }
  } else {
    if (mMode == PAINT) {
      if (buttonPressed) {
        if (event->buttons() & Qt::LeftButton)
          labelPoints(event->x(), event->y(), mRadius, mCurrentLabel, false);
        else
          labelPoints(event->x(), event->y(), mRadius, mCurrentLabel, true);
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
  }

  event->accept();
}

void Viewport::keyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_Escape:
      polygonPoints_.clear();  // start over again.#
      bufPolygonPoints_.assign(polygonPoints_);
      repaint();

      return;
    case Qt::Key_Delete:
      // delete last polygon point.

      if (mMode == POLYGON && polygonPoints_.size() > 0) {
        polygonPoints_.pop_back();

        bufPolygonPoints_.assign(polygonPoints_);
        repaint();
      }

      return;
    //    case Qt::Key_P:
    //      // Set plane parameters to car pose
    //      {
    //        planeThresholdNormal_ = 0;
    //        Eigen::Vector4f unit_vect(1.0, 0, 0, 0);
    //        auto normal_vect = points_[singleScanIdx_]->pose * unit_vect;
    //
    //        //        std::cout << "pose_matrix: " << std::endl << points_[singleScanIdx_]->pose << std::endl;
    //        //        std::cout << "normal_vect: " << normal_vect << std::endl;
    //
    //        planeNormal_[0] = normal_vect[0];
    //        planeNormal_[1] = normal_vect[1];
    //        planeNormal_[2] = normal_vect[2];
    //        planeDirectionNormal_ = 1.0;
    //        updateGL();
    //
    //        return;
    //      }
    // camera control
    case Qt::Key_C:
      if (points_.size() > 0) {
        if (points_.size() == 0) return;
        auto mat = conversion_ * points_[singleScanIdx_]->pose.inverse() * conversion_.inverse();
        mCamera->setMatrix(mat);
        updateGL();
      }
      return;
    case Qt::Key_W:
    case Qt::Key_A:
    case Qt::Key_S:
    case Qt::Key_D:

      if (event->isAutoRepeat()) return;
      //      std::cout << event->key() << std::endl;
      if (mMode == POLYGON && polygonPoints_.size() > 0) {  // ugly hack!
        event->ignore();
        return;
      }

      if (pressedkeys.empty()) {
        timer_.start(1 / 60);
      }
      pressedkeys.insert(event->key());
      GlCamera::KeyboardKey k = resolveKeyboardKey(event->key());
      if (mCamera->keyPressed(k, resolveKeyboardModifier(event->modifiers()))) {
        event->accept();
      } else {
        event->ignore();
      };
      return;
  }
  // handle event by parent:
  //  std::cout << event->key() << std::endl;
  event->ignore();
}

void Viewport::keyReleaseEvent(QKeyEvent* event) {
  switch (event->key()) {
    // camera control
    case Qt::Key_W:
    case Qt::Key_A:
    case Qt::Key_S:
    case Qt::Key_D:
      if (event->isAutoRepeat()) return;
      pressedkeys.erase(event->key());
      if (pressedkeys.empty()) {
        timer_.stop();
      }
      GlCamera::KeyboardKey k = resolveKeyboardKey(event->key());
      if (mCamera->keyReleased(k, resolveKeyboardModifier(event->modifiers()))) {
        event->accept();
      } else {
        event->ignore();
      };
      return;
  }
  event->ignore();
  /*std::cout << "myset contains:";
    for (std::set<int>::iterator it=pressedkeys.begin(); it!=pressedkeys.end(); ++it)
      std::cout << ' ' << *it;
    std::cout << '\n';*/
}

void Viewport::setTileInfo(float x, float y, float tileSize) {
  std::cout << x << ", " << y << ", " << tileSize << std::endl;
  tilePos_ = vec2(x, y);
  tileSize_ = tileSize;
}

std::vector<uint32_t> Viewport::getSelectedLabels() {
  std::vector<uint32_t> selected;

  if (points_.size() == 0 || labels_.size() == 0) return selected;

  //  std::cout << "called labelPoints(" << x << ", " << y << ", " << radius << ", " << new_label << ")" << std::flush;
  //  Stopwatch::tic();

  bool showSingleScan = drawingOption_["single scan"];
  bool showScanRange = drawingOption_["show scan range"];

  ScopedBinder<GlVertexArray> vaoBinder(vao_points_);
  ScopedBinder<GlProgram> programBinder(prgGetSelectedLabels_);
  ScopedBinder<GlTransformFeedback> feedbackBinder(tfSelectedLabels_);

  prgGetSelectedLabels_.setUniform(GlUniform<Eigen::Matrix4f>("pose", Eigen::Matrix4f::Identity()));
  if (points_.size() > singleScanIdx_)
    prgGetSelectedLabels_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[singleScanIdx_]->pose));

  prgGetSelectedLabels_.setUniform(GlUniform<int32_t>("width", width()));
  prgGetSelectedLabels_.setUniform(GlUniform<int32_t>("height", height()));

  prgGetSelectedLabels_.setUniform(GlUniform<bool>("removeGround", removeGround_));
  prgGetSelectedLabels_.setUniform(GlUniform<float>("groundThreshold", groundThreshold_));
  prgGetSelectedLabels_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
  prgGetSelectedLabels_.setUniform(GlUniform<float>("tileSize", tileSize_));
  prgGetSelectedLabels_.setUniform(GlUniform<bool>("showAllPoints", drawingOption_["show all points"]));
  prgGetSelectedLabels_.setUniform(GlUniform<int32_t>("heightMap", 1));

  prgGetSelectedLabels_.setUniform(GlUniform<bool>("planeRemovalNormal", planeRemovalNormal_));
  prgGetSelectedLabels_.setUniform(GlUniform<Eigen::Vector3f>("planeNormal", planeNormal_));
  prgGetSelectedLabels_.setUniform(GlUniform<float>("planeThresholdNormal", planeThresholdNormal_));
  prgGetSelectedLabels_.setUniform(GlUniform<float>("planeDirectionNormal", planeDirectionNormal_));
  prgGetSelectedLabels_.setUniform(GlUniform<bool>("hideLabeledInstances", drawingOption_["hide labeled instances"]));

  Eigen::Matrix4f plane_pose = Eigen::Matrix4f::Identity();
  plane_pose(0, 3) = tilePos_.x;
  plane_pose(1, 3) = tilePos_.y;
  plane_pose(2, 3) = points_[0]->pose(2, 3);
  if (drawingOption_["carAsBase"] && points_.size() > singleScanIdx_) {
    plane_pose = points_[singleScanIdx_]->pose;
    //    plane_pose.col(3) = points_[singleScanIdx_]->pose.col(3);
  }

  prgGetSelectedLabels_.setUniform(GlUniform<Eigen::Matrix4f>("plane_pose", plane_pose));

  Eigen::Matrix4f view_ = mCamera->matrix();

  if (drawingOption_["follow pose"]) {
    //    view_ = view_ * conversion_ * currentFrame_->pose.inverse() * conversion_.inverse();

    RoSeCamera dummy;
    dummy.setMatrix(conversion_ * points_[singleScanIdx_]->pose.inverse() * conversion_.inverse());
    view_ = view_ * dummy.matrix();
  }

  mvp_ = projection_ * view_ * conversion_;
  prgGetSelectedLabels_.setUniform(mvp_);
  prgGetSelectedLabels_.setUniform(GlUniform<int32_t>("numTriangles", numTriangles_));

  glActiveTexture(GL_TEXTURE0);
  texTriangles_.bind();

  glActiveTexture(GL_TEXTURE1);
  texMinimumHeightMap_.bind();

  glEnable(GL_RASTERIZER_DISCARD);

  uint32_t count = 0;
  uint32_t max_size = bufSelectedLabels_.size();
  uint32_t buffer_start = 0;
  uint32_t buffer_size = bufLabels_.size();

  if (showSingleScan) {
    buffer_start = scanInfos_[singleScanIdx_].start;
    buffer_size = scanInfos_[singleScanIdx_].size;
  } else if (showScanRange) {
    buffer_start = scanInfos_[scanRangeBegin_].start;
    buffer_size = scanInfos_[scanRangeEnd_].start + scanInfos_[scanRangeEnd_].size - scanInfos_[scanRangeBegin_].start;
  }

  while (count * max_size < buffer_size) {
    uint32_t size = std::min<uint32_t>(max_size, buffer_size - count * max_size);

    tfSelectedLabels_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, buffer_start + count * max_size, size);
    uint32_t num_captured = tfSelectedLabels_.end();

    std::vector<uint32_t> temp;
    bufSelectedLabels_.get(temp);
    temp.resize(num_captured);
    selected.insert(selected.end(), temp.begin(), temp.end());

    count++;
  }

  glDisable(GL_RASTERIZER_DISCARD);

  glActiveTexture(GL_TEXTURE0);
  texTriangles_.release();
  glActiveTexture(GL_TEXTURE1);
  texMinimumHeightMap_.release();
  //  std::cout << Stopwatch::toc() << " s." << std::endl;

  return selected;
}

void Viewport::labelPoints(int32_t x, int32_t y, float radius, uint32_t new_label, bool remove) {
  if (points_.size() == 0 || labels_.size() == 0) return;
  if (labelInstances_ && !instanceSelected_ && (instanceLabelingMode_ != 3)) {
    std::cout << "no instance selected!" << std::endl;
    return;
  }

  //  std::cout << "called labelPoints(" << x << ", " << y << ", " << radius << ", " << new_label << ")" << std::flush;
  //  Stopwatch::tic();

  bool showSingleScan = drawingOption_["single scan"];
  bool showScanRange = drawingOption_["show scan range"];

  ScopedBinder<GlVertexArray> vaoBinder(vao_points_);
  ScopedBinder<GlProgram> programBinder(prgUpdateLabels_);
  ScopedBinder<GlTransformFeedback> feedbackBinder(tfUpdateLabels_);

  prgUpdateLabels_.setUniform(GlUniform<Eigen::Matrix4f>("pose", Eigen::Matrix4f::Identity()));
  if (points_.size() > singleScanIdx_)
    prgUpdateLabels_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[singleScanIdx_]->pose));

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
  prgUpdateLabels_.setUniform(GlUniform<bool>("removeLabel", remove));
  prgUpdateLabels_.setUniform(GlUniform<bool>("hideLabeledInstances", drawingOption_["hide labeled instances"]));

  // set instance specific stuff.
  prgUpdateLabels_.setUniform(GlUniform<bool>("labelInstances", labelInstances_));
  if (labelInstances_) {
    prgUpdateLabels_.setUniform(GlUniform<int32_t>("instanceLabelingMode", instanceLabelingMode_));
    prgUpdateLabels_.setUniform(GlUniform<uint32_t>("selectedInstanceId", selectedInstanceId_));
    prgUpdateLabels_.setUniform(GlUniform<uint32_t>("selectedInstanceLabel", selectedInstanceLabel_));
    prgUpdateLabels_.setUniform(GlUniform<uint32_t>(
        "newInstanceId", (instanceLabelingMode_ == 4) ? new_label : maxInstanceIds_[selectedInstanceLabel_] + 1));
  }

  prgUpdateLabels_.setUniform(GlUniform<bool>("planeRemovalNormal", planeRemovalNormal_));
  prgUpdateLabels_.setUniform(GlUniform<Eigen::Vector3f>("planeNormal", planeNormal_));
  prgUpdateLabels_.setUniform(GlUniform<float>("planeThresholdNormal", planeThresholdNormal_));
  prgUpdateLabels_.setUniform(GlUniform<float>("planeDirectionNormal", planeDirectionNormal_));

  Eigen::Matrix4f plane_pose = Eigen::Matrix4f::Identity();
  plane_pose(0, 3) = tilePos_.x;
  plane_pose(1, 3) = tilePos_.y;
  plane_pose(2, 3) = points_[0]->pose(2, 3);
  if (drawingOption_["carAsBase"] && points_.size() > singleScanIdx_) {
    plane_pose = points_[singleScanIdx_]->pose;
    //    plane_pose.col(3) = points_[singleScanIdx_]->pose.col(3);
  }

  prgUpdateLabels_.setUniform(GlUniform<Eigen::Matrix4f>("plane_pose", plane_pose));

  Eigen::Matrix4f view_ = mCamera->matrix();

  if (drawingOption_["follow pose"]) {
    //    view_ = view_ * conversion_ * currentFrame_->pose.inverse() * conversion_.inverse();

    RoSeCamera dummy;
    dummy.setMatrix(conversion_ * points_[singleScanIdx_]->pose.inverse() * conversion_.inverse());
    view_ = view_ * dummy.matrix();
  }

  mvp_ = projection_ * view_ * conversion_;

  prgUpdateLabels_.setUniform(mvp_);

  if (mMode == Viewport::PAINT) prgUpdateLabels_.setUniform(GlUniform<int32_t>("labelingMode", 0));
  if (mMode == Viewport::POLYGON || labelInstances_) {
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

  // ignore single scan for joining.
  if (showSingleScan && !(labelInstances_ && instanceLabelingMode_ == 4)) {
    buffer_start = scanInfos_[singleScanIdx_].start;
    buffer_size = scanInfos_[singleScanIdx_].size;
  } else if (showScanRange) {
    buffer_start = scanInfos_[scanRangeBegin_].start;
    buffer_size = scanInfos_[scanRangeEnd_].start + scanInfos_[scanRangeEnd_].size - scanInfos_[scanRangeBegin_].start;
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

glow::GlCamera::KeyboardKey Viewport::resolveKeyboardKey(int key) {
  switch (key) {
    case Qt::Key_A:
      return glow::GlCamera::KeyboardKey::KeyA;
    case Qt::Key_B:
      return glow::GlCamera::KeyboardKey::KeyB;
    case Qt::Key_C:
      return glow::GlCamera::KeyboardKey::KeyC;
    case Qt::Key_D:
      return glow::GlCamera::KeyboardKey::KeyD;
    case Qt::Key_E:
      return glow::GlCamera::KeyboardKey::KeyE;
    case Qt::Key_F:
      return glow::GlCamera::KeyboardKey::KeyF;
    case Qt::Key_G:
      return glow::GlCamera::KeyboardKey::KeyG;
    case Qt::Key_H:
      return glow::GlCamera::KeyboardKey::KeyH;
    case Qt::Key_I:
      return glow::GlCamera::KeyboardKey::KeyI;
    case Qt::Key_J:
      return glow::GlCamera::KeyboardKey::KeyJ;
    case Qt::Key_K:
      return glow::GlCamera::KeyboardKey::KeyK;
    case Qt::Key_L:
      return glow::GlCamera::KeyboardKey::KeyL;
    case Qt::Key_M:
      return glow::GlCamera::KeyboardKey::KeyM;
    case Qt::Key_N:
      return glow::GlCamera::KeyboardKey::KeyN;
    case Qt::Key_O:
      return glow::GlCamera::KeyboardKey::KeyO;
    case Qt::Key_P:
      return glow::GlCamera::KeyboardKey::KeyP;
    case Qt::Key_Q:
      return glow::GlCamera::KeyboardKey::KeyQ;
    case Qt::Key_R:
      return glow::GlCamera::KeyboardKey::KeyR;
    case Qt::Key_S:
      return glow::GlCamera::KeyboardKey::KeyS;
    case Qt::Key_T:
      return glow::GlCamera::KeyboardKey::KeyT;
    case Qt::Key_U:
      return glow::GlCamera::KeyboardKey::KeyU;
    case Qt::Key_V:
      return glow::GlCamera::KeyboardKey::KeyV;
    case Qt::Key_W:
      return glow::GlCamera::KeyboardKey::KeyW;
    case Qt::Key_X:
      return glow::GlCamera::KeyboardKey::KeyX;
    case Qt::Key_Y:
      return glow::GlCamera::KeyboardKey::KeyY;
    case Qt::Key_Z:
      return glow::GlCamera::KeyboardKey::KeyZ;
    case Qt::Key_0:
      return glow::GlCamera::KeyboardKey::Key0;
    case Qt::Key_1:
      return glow::GlCamera::KeyboardKey::Key1;
    case Qt::Key_2:
      return glow::GlCamera::KeyboardKey::Key2;
    case Qt::Key_3:
      return glow::GlCamera::KeyboardKey::Key3;
    case Qt::Key_4:
      return glow::GlCamera::KeyboardKey::Key4;
    case Qt::Key_5:
      return glow::GlCamera::KeyboardKey::Key5;
    case Qt::Key_6:
      return glow::GlCamera::KeyboardKey::Key6;
    case Qt::Key_7:
      return glow::GlCamera::KeyboardKey::Key7;
    case Qt::Key_8:
      return glow::GlCamera::KeyboardKey::Key8;
    case Qt::Key_9:
      return glow::GlCamera::KeyboardKey::Key9;
    case Qt::Key_F1:
      return glow::GlCamera::KeyboardKey::KeyF1;
    case Qt::Key_F2:
      return glow::GlCamera::KeyboardKey::KeyF2;
    case Qt::Key_F3:
      return glow::GlCamera::KeyboardKey::KeyF3;
    case Qt::Key_F4:
      return glow::GlCamera::KeyboardKey::KeyF4;
    case Qt::Key_F5:
      return glow::GlCamera::KeyboardKey::KeyF5;
    case Qt::Key_F6:
      return glow::GlCamera::KeyboardKey::KeyF6;
    case Qt::Key_F7:
      return glow::GlCamera::KeyboardKey::KeyF7;
    case Qt::Key_F8:
      return glow::GlCamera::KeyboardKey::KeyF8;
    case Qt::Key_F9:
      return glow::GlCamera::KeyboardKey::KeyF9;
    case Qt::Key_F10:
      return glow::GlCamera::KeyboardKey::KeyF10;
    case Qt::Key_F11:
      return glow::GlCamera::KeyboardKey::KeyF11;
    case Qt::Key_F12:
      return glow::GlCamera::KeyboardKey::KeyF12;
    case Qt::Key_Escape:
      return glow::GlCamera::KeyboardKey::KeyEsc;
    case Qt::Key_Up:
      return glow::GlCamera::KeyboardKey::KeyUpArrow;
    case Qt::Key_Down:
      return glow::GlCamera::KeyboardKey::KeyDownArrow;
    case Qt::Key_Left:
      return glow::GlCamera::KeyboardKey::KeyLeftArrow;
    case Qt::Key_Right:
      return glow::GlCamera::KeyboardKey::KeyRightArrow;
    case Qt::Key_Space:
      return glow::GlCamera::KeyboardKey::KeySpace;
    case Qt::Key_Enter:
      return glow::GlCamera::KeyboardKey::KeyEnter;
    case Qt::Key_Return:
      return glow::GlCamera::KeyboardKey::KeyEnter;
    default:
      return glow::GlCamera::KeyboardKey::KeyNotSupported;
  }
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

glow::GlCamera::MouseButton Viewport::resolveMouseButtonFlip(Qt::MouseButtons button) {
  // currently only single button presses are supported.
  GlCamera::MouseButton btn = GlCamera::MouseButton::NoButton;
  if (flipMouseButtons) {
    if (button & Qt::LeftButton)
      btn = GlCamera::MouseButton::LeftButton;
    else if (button & Qt::RightButton)
      btn = GlCamera::MouseButton::MiddleButton;
    else if (button & Qt::MiddleButton)
      btn = GlCamera::MouseButton::RightButton;
  } else {
    if (button & Qt::LeftButton)
      btn = GlCamera::MouseButton::LeftButton;
    else if (button & Qt::RightButton)
      btn = GlCamera::MouseButton::RightButton;
    else if (button & Qt::MiddleButton)
      btn = GlCamera::MouseButton::MiddleButton;
  }
  return btn;
}

void Viewport::centerOnCurrentTile() {
  // have to convert from robotic coordinate system to the opengl system.
  if (points_.size() == 0) return;

  Eigen::Vector4f t = points_[0]->pose.col(3);

  mCamera->lookAt(-tilePos_.y + 20, t.z() + 25, -tilePos_.x + 20, -tilePos_.y, t.z(), -tilePos_.x);
  updateGL();
}

void Viewport::setPlaneRemoval(bool value) {
  planeRemoval_ = value;
  updateGL();
}

void Viewport::setPlaneRemovalParams(float threshold, int32_t dim, float direction) {
  planeThreshold_ = threshold;
  planeDimension_ = dim;
  planeDirection_ = direction;
  updateGL();
}

void Viewport::setPlaneRemovalNormal(bool value) {
  planeRemovalNormal_ = value;
  updateGL();
}

void Viewport::setPlaneRemovalNormalParams(float threshold, float A1, float A2, float A3, float direction) {
  planeThresholdNormal_ = threshold;
  Eigen::Vector4f unit_vect(1.0, 0, 0, 0);
  //  auto rotX = glow::glRotateX(A1 * PI / 180);
  //  auto rotY = glow::glRotateY(A2 * PI / 180);
  //  auto rotZ = glow::glRotateZ(A3 * PI / 180);

  //  auto normal_vect = rotX * rotY * rotZ * unit_vect;
  const double PI = std::acos(-1);
  float theta = A1 * PI / 180;
  float phi = A2 * PI / 180;

  //  Eigen::Matrix4f R = Eigen::Matrix4f::Identity();
  //  R(0, 0) = std::sin(theta) * std::cos(phi);
  //  R(0, 1) = std::cos(theta) * std::cos(phi);
  //  R(0, 2) = -std::sin(phi);
  //
  //  R(1, 0) = std::sin(theta) * std::sin(phi);
  //  R(1, 1) = std::cos(theta) * std::sin(phi);
  //  R(1, 2) = std::cos(phi);
  //
  //  R(2, 0) = std::cos(theta);
  //  R(2, 1) = -std::sin(phi);
  //  R(2, 2) = 0;
  //
  //  auto rotX = glow::glRotateX(theta);
  //  auto rotY = glow::glRotateY(phi);
  //  R = rotX * rotY;
  //  auto normal_vect = R * Eigen::Vector4f(1, 0, 0, 0);

  float a = std::tan(theta);
  float b = std::tan(phi);
  Eigen::Vector3f n(a, b, 1);
  n.normalize();

  //  std::cout << "rotation matrix: " << std::endl << rotX * rotY * rotZ << std::endl;
  //  std::cout << "normal_vect: " << normal_vect << std::endl;

  //  planeNormal_[0] = normal_vect[0];
  //  planeNormal_[1] = normal_vect[1];
  //  planeNormal_[2] = normal_vect[2];
  planeNormal_ = n;  // normal_vect.head(3);
  planeDirectionNormal_ = direction;
  updateGL();
}

void Viewport::setFlipMouseButtons(bool value) { flipMouseButtons = value; }

std::vector<std::string> Viewport::getCameraNames() const {
  std::vector<std::string> keys;
  for (auto it = cameras_.begin(); it != cameras_.end(); ++it) keys.push_back(it->first);
  return keys;
}

std::map<std::string, std::shared_ptr<glow::GlCamera>> Viewport::getCameras() const { return cameras_; }

void Viewport::setCamera(const std::shared_ptr<glow::GlCamera>& cam) {
  Eigen::Matrix4f m = mCamera->matrix();
  mCamera = cam;
  mCamera->setMatrix(m);
}

void Viewport::setCameraProjection(const CameraProjection& proj) {
  projectionMode_ = proj;
  resizeGL(width(), height());
  updateGL();
}

void Viewport::setCameraByName(const std::string& name) {
  if (cameras_.find(name) == cameras_.end()) return;

  setCamera(cameras_[name]);
}

void Viewport::setInstanceableLabels(const std::vector<uint32_t>& labels) { instanceableLabels_ = labels; }

void Viewport::labelInstances(bool value) {
  labelInstances_ = value;
  if (labelInstances_) {
    updateBoundingBoxes();
    updateInstanceSelectionMap();
  }

  updateGL();
}

void Viewport::setMaximumInstanceIds(const std::map<uint32_t, uint32_t>& maxInstanceIds) {
  maxInstanceIds_ = maxInstanceIds;
}

std::map<uint32_t, uint32_t> Viewport::getMaximumInstanceIds() const { return maxInstanceIds_; }

void Viewport::setInstanceLabelingMode(int32_t value) {
  instanceLabelingMode_ = value;
  abortPolygonSelection();

  updateGL();
}

void Viewport::setInstanceSelectionMode(bool value) {
  instanceSelectionMode_ = value;
  if (instanceLabelingMode_) abortPolygonSelection();

  if (instanceSelectionMode_ && instanceLabelingMode_ != 4) {
    instanceSelected_ = false;
    selectedInstanceId_ = 0;
    selectedInstanceLabel_ = 0;
  }
}

void Viewport::updateBoundingBoxes() {
  if (points_.size() == 0) return;
  std::cout << "updating bounding boxes. " << std::flush;

  glow::GlBuffer<vec4> bufReadPoints{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STREAM_READ};
  glow::GlBuffer<uint32_t> bufReadLabels{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STREAM_READ};
  glow::GlBuffer<vec2> bufReadIndexes{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STREAM_READ};

  bufReadPoints.resize(maxPointsPerScan_);
  bufReadLabels.resize(maxPointsPerScan_);
  bufReadIndexes.resize(maxPointsPerScan_);

  std::vector<vec4> points(bufReadLabels.size());
  std::vector<uint32_t> labels(bufReadLabels.size());
  std::vector<vec2> indexes(bufReadIndexes.size());

  uint32_t count = 0;
  uint32_t max_size = bufReadLabels.size();
  uint32_t buffer_size = bufLabels_.size();

  labeledCount_ = 0;

  bboxes_static_.clear();
  bboxes_moving_.clear();

  std::cout << "Collecting min/max: [" << std::flush;
  float progress = 0.1;
  while (count * max_size < bufLabels_.size()) {
    if (count * max_size > progress * bufLabels_.size()) {
      std::cout << "=" << std::flush;
      progress += 0.1;
    }

    uint32_t size = std::min<uint32_t>(max_size, buffer_size - count * max_size);

    bufLabels_.copyTo(count * max_size, size, bufReadLabels, 0);
    bufPoints_.copyTo(count * max_size, size, bufReadPoints, 0);
    bufScanIndexes_.copyTo(count * max_size, size, bufReadIndexes, 0);

    bufReadLabels.get(labels, 0, size);
    bufReadPoints.get(points, 0, size);
    bufReadIndexes.get(indexes, 0, size);

    for (uint32_t i = 0; i < size; ++i) {
      // store only info for real instances.
      uint32_t instanceId = ((labels[i] >> 16) & uint32_t(0xFFFF));
      uint32_t label = (labels[i] & uint32_t(0xFFFF));
      uint32_t t = uint32_t(indexes[i].x);
      bool is_moving = (label > 200);
      maxInstanceIds_[label] = std::max(maxInstanceIds_[label], instanceId);

      if (instanceId > 0) {
        if (is_moving) {
          if (bboxes_moving_.find(labels[i]) == bboxes_moving_.end())
            bboxes_moving_[labels[i]] = std::map<uint32_t, BoundingBox>();

          if (bboxes_moving_[labels[i]].find(t) == bboxes_moving_[labels[i]].end()) {
            bboxes_moving_[labels[i]][t] = BoundingBox();
            bboxes_moving_[labels[i]][t].position_yaw = bboxes_moving_[labels[i]][t].size_id = points[i];
          } else {
            bboxes_moving_[labels[i]][t].position_yaw.x =
                std::min(points[i].x, bboxes_moving_[labels[i]][t].position_yaw.x);
            bboxes_moving_[labels[i]][t].position_yaw.y =
                std::min(points[i].y, bboxes_moving_[labels[i]][t].position_yaw.y);
            bboxes_moving_[labels[i]][t].position_yaw.z =
                std::min(points[i].z, bboxes_moving_[labels[i]][t].position_yaw.z);

            bboxes_moving_[labels[i]][t].size_id.x = std::max(points[i].x, bboxes_moving_[labels[i]][t].size_id.x);
            bboxes_moving_[labels[i]][t].size_id.y = std::max(points[i].y, bboxes_moving_[labels[i]][t].size_id.y);
            bboxes_moving_[labels[i]][t].size_id.z = std::max(points[i].z, bboxes_moving_[labels[i]][t].size_id.z);
          }

        } else {
          if (bboxes_static_.find(labels[i]) == bboxes_static_.end()) {
            bboxes_static_[labels[i]] = BoundingBox();
            bboxes_static_[labels[i]].position_yaw = bboxes_static_[labels[i]].size_id = points[i];
          } else {
            bboxes_static_[labels[i]].position_yaw.x = std::min(points[i].x, bboxes_static_[labels[i]].position_yaw.x);
            bboxes_static_[labels[i]].position_yaw.y = std::min(points[i].y, bboxes_static_[labels[i]].position_yaw.y);
            bboxes_static_[labels[i]].position_yaw.z = std::min(points[i].z, bboxes_static_[labels[i]].position_yaw.z);

            bboxes_static_[labels[i]].size_id.x = std::max(points[i].x, bboxes_static_[labels[i]].size_id.x);
            bboxes_static_[labels[i]].size_id.y = std::max(points[i].y, bboxes_static_[labels[i]].size_id.y);
            bboxes_static_[labels[i]].size_id.z = std::max(points[i].z, bboxes_static_[labels[i]].size_id.z);
          }
        }
      }
    }
    count++;
  }

  std::cout << "]" << std::flush;

  std::vector<vec4> position_yaw;
  std::vector<vec4> size_id;
  id2idx_.clear();

  // compute bounding boxes from temp information.
  for (auto it = bboxes_static_.begin(); it != bboxes_static_.end(); ++it) {
    Eigen::Vector3f minimum(it->second.position_yaw.x, it->second.position_yaw.y, it->second.position_yaw.z);
    Eigen::Vector3f maximum(it->second.size_id.x, it->second.size_id.y, it->second.size_id.z);
    Eigen::Vector3f size = (maximum - minimum);
    Eigen::Vector3f mid = (minimum + 0.5 * size);

    it->second.position_yaw.x = mid.x();
    it->second.position_yaw.y = mid.y();
    it->second.position_yaw.z = mid.z();
    it->second.position_yaw.w = 0.0f;
    it->second.size_id.x = size.x();
    it->second.size_id.y = size.y();
    it->second.size_id.z = size.z();
  }

  // for each labeled instance:
  for (auto iit = bboxes_moving_.begin(); iit != bboxes_moving_.end(); ++iit) {
    // for each timestep:
    for (auto it = iit->second.begin(); it != iit->second.end(); ++it) {
      Eigen::Vector3f minimum(it->second.position_yaw.x, it->second.position_yaw.y, it->second.position_yaw.z);
      Eigen::Vector3f maximum(it->second.size_id.x, it->second.size_id.y, it->second.size_id.z);
      Eigen::Vector3f size = (maximum - minimum);
      Eigen::Vector3f mid = (minimum + 0.5 * size);

      it->second.position_yaw.x = mid.x();
      it->second.position_yaw.y = mid.y();
      it->second.position_yaw.z = mid.z();
      it->second.position_yaw.w = 0.0f;
      it->second.size_id.x = size.x();
      it->second.size_id.y = size.y();
      it->second.size_id.z = size.z();
    }
  }

  // ... and fill buffer.
  fillBoundingBoxBuffers();
  std::cout << " finished." << std::endl;

  updateGL();
}

void Viewport::fillBoundingBoxBuffers() {
  id2idx_.clear();

  std::vector<vec4> position_yaw;
  std::vector<vec4> size_id;
  std::vector<uint32_t> instance_label;

  for (auto it = bboxes_static_.begin(); it != bboxes_static_.end(); ++it) {
    id2idx_[it->first] = position_yaw.size();
    position_yaw.push_back(it->second.position_yaw);
    size_id.push_back(it->second.size_id);
    instance_label.push_back(it->first);
  }

  if (drawingOption_["show all moving instances"]) {
    for (auto it = bboxes_moving_.begin(); it != bboxes_moving_.end(); ++it) {
      for (auto iit = it->second.begin(); iit != it->second.end(); ++iit) {
        id2idx_[it->first] = position_yaw.size();
        position_yaw.push_back(iit->second.position_yaw);
        size_id.push_back(iit->second.size_id);
        instance_label.push_back(it->first);
      }
    }
  } else {
    for (auto it = bboxes_moving_.begin(); it != bboxes_moving_.end(); ++it) {
      if (it->second.find(singleScanIdx_) != it->second.end()) {
        id2idx_[it->first] = position_yaw.size();
        position_yaw.push_back(it->second.find(singleScanIdx_)->second.position_yaw);
        size_id.push_back(it->second.find(singleScanIdx_)->second.size_id);
        instance_label.push_back(it->first);
      }
    }
  }

  bufBboxPositionsYaw_.assign(position_yaw);
  bufBboxSizeIds_.assign(size_id);
  bufBboxInstanceLabels_.assign(instance_label);

  glow::_CheckGlError(__FILE__, __LINE__);
}
