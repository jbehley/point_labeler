#include "Viewport.h"
#include "data/Math.h"
#include "data/draw_utils.h"

#include <glow/ScopedBinder.h>
#include <glow/glutil.h>
#include <algorithm>
#include <chrono>
#include "rv/Stopwatch.h"

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
      texLabelColors_(256, 1, TextureFormat::RGB) {
  connect(&timer_, &QTimer::timeout, [this]() { this->updateGL(); });

  //  setMouseTracking(true);

  drawing_options_["coordinate axis"] = true;

  conversion_ = RoSe2GL::matrix;

  uint32_t max_size = maxScans_ * maxPointsPerScan_;

  bufPoses_.resize(maxScans_);
  bufPoints_.resize(max_size);
  bufRemissions_.resize(max_size);
  bufVisible_.resize(max_size);
  bufLabels_.resize(max_size);

  std::vector<std::string> feedback_varyings{"projected_point"};

  bufProjectedPoints_.resize(maxPointsPerScan_);
  tfProjectedPoints_.attach(feedback_varyings, bufProjectedPoints_);

  bufUpdatedLabels_.resize(maxPointsPerScan_);
  std::vector<std::string> update_varyings{"out_label"};
  tfUpdateLabels_.attach(update_varyings, bufUpdatedLabels_);

  initPrograms();
  initVertexBuffers();

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

  prgProjectPoints_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/project_points.vert"));
  prgProjectPoints_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/empty.frag"));
  prgProjectPoints_.attach(tfProjectedPoints_);
  prgProjectPoints_.link();

  prgUpdateLabels_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/update_labels.vert"));
  prgUpdateLabels_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/empty.frag"));
  prgUpdateLabels_.attach(tfUpdateLabels_);
  prgUpdateLabels_.link();
}

void Viewport::initVertexBuffers() {
  vao_points_.setVertexAttribute(0, bufPoints_, 3, AttributeType::FLOAT, false, sizeof(Point3f), nullptr);
  vao_points_.setVertexAttribute(1, bufRemissions_, 1, AttributeType::FLOAT, false, sizeof(float), nullptr);
  vao_points_.setVertexAttribute(2, bufLabels_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);
  vao_points_.setVertexAttribute(3, bufVisible_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);
}

/** \brief set axis fixed (x = 1, y = 2, z = 3) **/
void Viewport::setFixedAxis(AXIS axis) { mAxis = axis; }

void Viewport::setPoints(const std::vector<PointcloudPtr>& p, std::vector<LabelsPtr>& l) {
  std::cout << "Setting points..." << std::flush;
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

    std::vector<Laserscan*> needsDelete;
    std::vector<Laserscan*>::iterator end =
        std::set_difference(before.begin(), before.end(), after.begin(), after.end(), needsDelete.begin());

    for (auto it = needsDelete.begin(); it != end; ++it) bufferContent_.erase(*it);
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

      std::vector<uint32_t> visible(num_points, 1);  // TODO: use visibilities by toggled labels.

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

  std::cout << "Loaded " << loadedScans << " of total " << points_.size() << " scans." << std::endl;
  std::cout << "memcpy: " << memcpy_time << " s / " << total_time << " s." << std::endl;

  updateGL();
}

void Viewport::setRadius(float value) { mRadius = value; }

void Viewport::setLabel(uint32_t label) { mCurrentLabel = label; }

void Viewport::setLabelColors(const std::map<uint32_t, glow::GlColor>& colors) {
  mLabelColors = colors;

  std::vector<vec3> label_colors(256);
  for (auto it = mLabelColors.begin(); it != mLabelColors.end(); ++it)
    label_colors[it->first] = vec3(it->second.R, it->second.G, it->second.B);

  label_colors[0] = vec3(0, 155, 0);
  texLabelColors_.assign(PixelFormat::RGB, PixelType::UNSIGNED_INT, &label_colors[0]);
}

void Viewport::setPointSize(int value) {
  pointSize_ = value;
  updateGL();
}

void Viewport::setMode(MODE mode) {
  mMode = mode;
  std::cout << "mode is now paint. " << std::endl;
  updateGL();
}

void Viewport::setFlags(int32_t flags) { mFlags = flags; }

void Viewport::setOverwrite(bool value) {
  if (value)
    mFlags = mFlags | FLAG_OVERWRITE;
  else
    mFlags = mFlags & ~FLAG_OVERWRITE;
}

void Viewport::setFilteredLabels(const std::vector<uint32_t>& labels) {
  mFilteredLabels = labels;

  // TODO update visibility via SHADER (like the label update.)!
  updateGL();
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

//  updateProjections();
}

void Viewport::paintGL() {
  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPointSize(pointSize_);

  model_ = Eigen::Matrix4f::Identity();
  view_ = mCamera.matrix();

  mvp_ = projection_ * view_ * conversion_;

  if (drawing_options_["coordinate axis"]) {
    // coordinateSytem_->pose = Eigen::Matrix4f::Identity();
    ScopedBinder<GlProgram> program_binder(prgDrawPose_);
    ScopedBinder<GlVertexArray> vao_binder(vao_no_points_);

    prgDrawPose_.setUniform(mvp_);
    prgDrawPose_.setUniform(GlUniform<Eigen::Matrix4f>("pose", Eigen::Matrix4f::Identity()));
    prgDrawPose_.setUniform(GlUniform<float>("size", 1.0f));

    glDrawArrays(GL_POINTS, 0, 1);
  }

  if (points_.size() > 0) {
    ScopedBinder<GlProgram> program_binder(prgDrawPoints_);
    ScopedBinder<GlVertexArray> vao_binder(vao_points_);
    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.bind();

    for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) {
      mvp_ = projection_ * view_ * conversion_ * it->first->pose;
      prgDrawPoints_.setUniform(mvp_);

      glDrawArrays(GL_POINTS, it->second.index * maxPointsPerScan_, it->second.size);
    }

    texLabelColors_.release();
  }
}

void Viewport::mousePressEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (event->modifiers() == Qt::ControlModifier || (mMode != PAINT)) {
    if (mCamera.mousePressed(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                             resolveKeyboardModifier(event->modifiers()))) {
      timer_.start(1. / 30.);
      mChangeCamera = true;
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
  }
  //  else if ((mCurrentPaintMode & PAINT_FILLPOLYGON) && (e->buttons() & Qt::LeftButton))
  //  {
  //    vertices.push_back(RoSe::Vector2(e->x(), e->y()));
  //
  //    updateGL();
  //  }

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
  }
  //  if (mSelectionMode && !((mCurrentPaintMode & PAINT_BRUSH)
  //      || (mCurrentPaintMode & PAINT_FILLPOLYGON)) && (e->button()
  //      == Qt::LeftButton) && e->modifiers() != Qt::ControlModifier)
  //  {
  //    curr_selected_point = getClickedPoint(e->x(), e->y());
  //    if (curr_selected_point != 0) emit pointSelected();
  //
  //    //    mUpdateDisplayList = true;
  //    //    updateGL();
  //  }
  //  else if ((mCurrentPaintMode & PAINT_FILLPOLYGON) && (e->button()
  //      == Qt::LeftButton))
  //  {
  //    std::cout << "Mouse released at (" << e->x() << ", " << e->y() << ")"
  //        << std::endl;
  //    /** append point to polygon list **/
  //    updateGL();
  //  }
  //  else if ((mCurrentPaintMode & PAINT_FILLPOLYGON) && (e->button()
  //      == Qt::RightButton))
  //  {
  //    /** (1) determine winding order, really bullet proof is this method not, but it should work in most
  //    situations.
  //    **/
  //    if (vertices.size() >= 3)
  //    {
  //      RoSe::Vector2 midpoint;
  //
  //      for (unsigned int i = 0; i < vertices.size(); ++i)
  //        midpoint += vertices[i];
  //
  //      midpoint /= vertices.size();
  //
  //      /** the coordinate systems origin is in the top-left corner, so that the y axis is flipped. **/
  //      int order = Math::halfPlaneTest(vertices[0], vertices[1], midpoint);
  //
  //      if (order > 0) // clockwise order; we have to reorder the vertices.
  //      {
  //        std::cout << "reordering the vertices." << std::endl;
  //
  //        std::cout << "BEFORE REORDERING:" << std::endl;
  //        for (unsigned int i = 0; i < vertices.size(); ++i)
  //          std::cout << vertices[i] << std::endl;
  //
  //        std::vector<RoSe::Vector2> temp;
  //        for (int i = vertices.size() - 1; i >= 0; --i)
  //          temp.push_back(vertices[i]);
  //
  //        vertices = temp;
  //
  //        std::cout << "AFTER REORDERING: " << std::endl;
  //        for (unsigned int i = 0; i < vertices.size(); ++i)
  //          std::cout << vertices[i] << std::endl;
  //
  //        assert(temp.size() == vertices.size());
  //
  //      }
  //      std::list<std::vector<RoSe::Vector2> >::iterator pit =
  //          projected_points.begin();
  //      /** (2) label points **/
  //      for (std::list<LabeledLaserscan>::iterator it = mFirstLaserscan; it
  //          != mLastLaserscan; ++it, ++pit)
  //        labelPoints(*it, *pit, vertices, mCurrentLabel);
  //    }
  //    vertices.clear();
  //
  //    mUpdateDisplayList = true;
  //    updateGL();
  //  }
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
  }
  //  else if ((mCurrentPaintMode & PAINT_FILLPOLYGON) && (e->buttons()
  //      & Qt::LeftButton))
  //  {
  //    /** update last point **/
  //    RoSe::Vector2& lastPoint = vertices.back();
  //    lastPoint[0] = e->x();
  //    lastPoint[1] = e->y();
  //    updateGL();
  //  }
  event->accept();
}

void Viewport::keyPressEvent(QKeyEvent*) {}

void Viewport::labelPoints(int32_t x, int32_t y, float radius, uint32_t new_label) {
  if (points_.size() == 0 || labels_.size() == 0) return;

  std::cout << "called labelPoints(" << x << ", " << y << ", " << radius << ", " << new_label << ")" << std::endl;
  Stopwatch::tic();

  ScopedBinder<GlVertexArray> vaoBinder(vao_points_);
  ScopedBinder<GlProgram> programBinder(prgUpdateLabels_);
  ScopedBinder<GlTransformFeedback> feedbackBinder(tfUpdateLabels_);

  prgUpdateLabels_.setUniform(GlUniform<vec2>("window_pos", glow::vec2(x, y)));
  prgUpdateLabels_.setUniform(GlUniform<int32_t>("width", width()));
  prgUpdateLabels_.setUniform(GlUniform<int32_t>("height", height()));
  prgUpdateLabels_.setUniform(GlUniform<float>("radius", radius));
  prgUpdateLabels_.setUniform(GlUniform<uint32_t>("new_label", new_label));

  glEnable(GL_RASTERIZER_DISCARD);

  for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) {
    mvp_ = projection_ * mCamera.matrix() * conversion_ * it->first->pose;

    prgUpdateLabels_.setUniform(mvp_);

    tfUpdateLabels_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, it->second.index * maxPointsPerScan_, it->second.size);
    tfUpdateLabels_.end();

    bufUpdatedLabels_.copyTo(bufLabels_, it->second.index * maxPointsPerScan_);

  }

  glDisable(GL_RASTERIZER_DISCARD);

  std::cout << Stopwatch::toc() << " s." << std::endl;


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
