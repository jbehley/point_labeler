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
  prgProjectPoints_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgProjectPoints_.attach(tfProjectedPoints_);
  prgProjectPoints_.link();

  prgUpdateLabels_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/update_labels.vert"));
  prgUpdateLabels_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgUpdateLabels_.attach(tfUpdateLabels_);
  prgUpdateLabels_.link();
}

void Viewport::initVertexBuffers() {
  vao_points_.setVertexAttribute(0, bufPoints_, 3, AttributeType::FLOAT, false, sizeof(Point3f), nullptr);
  vao_points_.setVertexAttribute(1, bufRemissions_, 1, AttributeType::FLOAT, false, sizeof(float), nullptr);
  vao_points_.setVertexAttribute(2, bufLabels_, 1, AttributeType::FLOAT, false, sizeof(uint32_t), nullptr);
  vao_points_.setVertexAttribute(3, bufVisible_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);
}

/** \brief set axis fixed (x = 1, y = 2, z = 3) **/
void Viewport::setFixedAxis(AXIS axis) {
  mAxis = axis;
}

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

void Viewport::setRadius(float value) {
  mRadius = value;
}

void Viewport::setLabel(uint32_t label) {
  mCurrentLabel = label;
}

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

void Viewport::setFlags(int32_t flags) {
  mFlags = flags;
}

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

  updateProjections();
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
  //
  //    glMatrixMode(GL_MODELVIEW);
  //    glLoadIdentity();
  //    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  //
  //    Eigen::Matrix4f view = mCamera.matrix();
  //    glMultMatrixf(view.data());
  //
  //
  //  glPushMatrix();
  //
  //  glMultMatrixf(glow::RoSe2GL::matrix.data());
  //
  //  glBegin(GL_LINES);
  //  glColor3f(1.0f, 0.0f, 0.0f);
  //  glVertex3f(0.0f, 0.0f, 0.0f);
  //  glVertex3f(5.0f, 0.0f, 0.0f);
  //  glColor3f(0.0f, 1.0f, 0.0f);
  //  glVertex3f(0.0f, 0.0f, 0.0f);
  //  glVertex3f(0.0f, 5.0f, 0.0f);
  //  glColor3f(0.0f, 0.0f, 1.0f);
  //  glVertex3f(0.0f, 0.0f, 0.0f);
  //  glVertex3f(0.0f, 0.0f, 5.0f);
  //  glEnd();
  //
  //  glPopMatrix();
  //
  //  if (points != 0 && labels != 0) drawPoints(*points, *labels);
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

      //    vf.update();
      /** update the projected coordinates. TODO: only if pose changes. **/
      //      updateProjections();

      return;
    }
  } else if (mMode == PAINT) {
    buttonPressed = false;
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

// void Viewport::drawPoints(const std::vector<Point3f>& points, const std::vector<uint32_t>& labels) {
//  glPushMatrix();
//  glMultMatrixf(RoSe2GL::matrix.data());
//
//  glPointSize(pointSize_);
//  glColor3fv(GlColor::BLACK);
//
//  glBegin(GL_POINTS);
//  for (uint32_t i = 0; i < points.size(); ++i) {
//    bool found = false;
//
//    for (uint32_t j = 0; j < mFilteredLabels.size(); ++j) {
//      if (mFilteredLabels[j] == labels[i]) {
//        found = true;
//        break;
//      }
//    }
//
//    if (!found && mFilteredLabels.size() > 0) continue;
//
//    if (mLabelColors.find(labels[i]) == mLabelColors.end())
//      glColor3fv(GlColor::BLACK);
//    else
//      glColor3fv(mLabelColors[labels[i]]);
//    glVertex3fv(&points[i].x);
//  }
//  glEnd();
//
//  glPopMatrix();
//}

// uniform int width;
// uniform int height;
// uniform vec2 window_pos;
// uniform float radius;
// uniform uint new_label;

void Viewport::labelPoints(int32_t x, int32_t y, float radius, uint32_t new_label) {
  if (points_.size() == 0 || labels_.size() == 0) return;

  std::cout << "called labelPoints" << std::endl;
  Stopwatch::tic();

  ScopedBinder<GlVertexArray> vaoBinder(vao_points_);
  ScopedBinder<GlTransformFeedback> feedbackBinder(tfUpdateLabels_);
  ScopedBinder<GlProgram> programBinder(prgUpdateLabels_);

  prgUpdateLabels_.setUniform(GlUniform<vec2>("window_pos", glow::vec2(x, y)));
  prgUpdateLabels_.setUniform(GlUniform<int32_t>("width", width()));
  prgUpdateLabels_.setUniform(GlUniform<int32_t>("height", height()));
  prgUpdateLabels_.setUniform(GlUniform<float>("radius", radius));
  prgUpdateLabels_.setUniform(GlUniform<uint32_t>("new_label", new_label));

  glEnable(GL_RASTERIZER_DISCARD);

  for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) {
    mvp_ = projection_ * mCamera.matrix() * conversion_ * it->first->pose;
    prgUpdateLabels_.setUniform(mvp_);

    //    tfUpdateLabels_.setBufferOffset(bufLabels_, it->second.index * maxPointsPerScan_);

    tfUpdateLabels_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, it->second.index * maxPointsPerScan_, it->second.size);
    tfUpdateLabels_.end();

    bufUpdatedLabels_.copyTo(0, maxPointsPerScan_, bufLabels_, it->second.index * maxPointsPerScan_);
  }

  glDisable(GL_RASTERIZER_DISCARD);

  std::cout << Stopwatch::toc() << " s." << std::endl;

  //  float radius2 = radius * radius;
  //
  //  //  float min_distance2 = min_distance * min_distance;
  //  //  float max_distance2 = max_distance * max_distance;
  //
  //  // TODO: use quadtree to accelerate search.
  //
  //  for (uint32_t i = 0; i < projectedPoints_.size(); ++i) {
  //    //    Point3f& p = scan[i].pos;
  //    //    Point3f global_pos = scan.pose()(scan[i].pos);
  //    //    float distance = p.x * p.x + p.y * p.y + p.z * p.z;
  //    //
  //    //    Label label = labels[i];
  //    //    if (do_mapping && mapping.find(label) != mapping.end()) label = mapping[label];
  //    //    if (do_filtering && filter.find(label) == filter.end()) continue;
  //    //
  //    //    if (mFilterPointsDistance && (distance > max_distance2 || distance < min_distance2)) continue;
  //    //    if (!mShowLabeledPoints && label != 0) continue;
  //
  //    //      if (!vf.isInside((*points_[t])[i].x, (*points_[t])[i].y, (*points_[t])[i].z)) continue; // TODO::
  //    //      Culling!
  //
  //    bool found = false;
  //
  //    uint32_t& label = (*labels_[projectedPoints_[i].timestamp])[projectedPoints_[i].index];
  //
  //    for (uint32_t j = 0; j < mFilteredLabels.size(); ++j) {
  //      if (mFilteredLabels[j] == label) {
  //        found = true;
  //        break;
  //      }
  //    }
  //
  //    if (!found && mFilteredLabels.size() > 0) continue;
  //
  //    double dx = projectedPoints_[i].pos.x - x;
  //    double dy = projectedPoints_[i].pos.y - y;
  //    if (dx * dx + dy * dy < radius2) {
  //      if ((label == 0) || (mFlags & FLAG_OVERWRITE)) {
  //        label = new_label;
  //      }
  //    }
  //  }

  emit labelingChanged();
}

void Viewport::updateProjections() {
  Stopwatch::tic();
  ScopedBinder<GlVertexArray> vaoBinder(vao_points_);
  ScopedBinder<GlTransformFeedback> feedbackBinder(tfProjectedPoints_);
  ScopedBinder<GlProgram> programBinder(prgProjectPoints_);

  projectedPoints_.clear();

  uint32_t numPointsAll = 0;
  glEnable(GL_RASTERIZER_DISCARD);
  std::vector<vec3> projectedPointsScan(maxPointsPerScan_);
  for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) {
    mvp_ = projection_ * mCamera.matrix() * conversion_ * it->first->pose;
    prgDrawPoints_.setUniform(mvp_);

    tfProjectedPoints_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, it->second.index * maxPointsPerScan_, it->second.size);
    bufProjectedPoints_.resize(tfProjectedPoints_.end());

    bufProjectedPoints_.get(projectedPointsScan);
    for (uint32_t i = 0; i < projectedPointsScan.size(); ++i) {
      if (projectedPointsScan[i].x < -1.0f || projectedPointsScan[i].x > 1.0f || projectedPointsScan[i].y < -1.0f ||
          projectedPointsScan[i].y > 1.0f)
        continue;

      ProjectedPoint p;
      p.pos.x = 0.5 * (projectedPointsScan[i].x + 1.0f) * width();
      p.pos.y = height() - 0.5 * (projectedPointsScan[i].y + 1.0f);
      p.timestamp = it->second.index;
      p.index = i;
      projectedPoints_.push_back(p);
    }

    numPointsAll += projectedPointsScan.size();
  }

  glDisable(GL_RASTERIZER_DISCARD);

  std::cout << "Projected " << numPointsAll << " in " << Stopwatch::toc() << " s" << std::endl;
  //  if (points == 0 || labels == 0) return;
  //
  //  glPushMatrix();
  //  glMultMatrixf(glow::RoSe2GL::matrix.data());
  //
  //  GLint viewport[4];
  //  GLdouble modelMatrix[16];
  //  GLdouble projectionMatrix[16];
  //
  //  glGetIntegerv(GL_VIEWPORT, viewport);
  //  glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
  //  glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
  //
  //  projected_points.resize(points->size());
  //  for (uint32_t i = 0; i < points->size(); ++i) {
  //    double dummy = 0.0;
  //
  //    const Point3f& p = (*points)[i];
  //
  //    double x, y;
  //
  //    gluProject(p.x, p.y, p.z, modelMatrix, projectionMatrix, viewport, &(x), &(y), &(dummy));
  //    projected_points[i].x = x;
  //    projected_points[i].y = y;
  //    /** the coordinate systems origin is in the top-left corner! **/
  //    projected_points[i].y = viewport[3] - projected_points[i].y;
  //  }
  //
  //  glPopMatrix();
}

// std::vector<Point3f*> Viewport::getSelectedCylinderPoints(const QPoint& pos) {
//  if (cylinders == 0) return std::vector<Point3f*>();
//  if (mAxis == XYZ) return std::vector<Point3f*>();
//  if (pos.x() >= width() || pos.y() >= height() || pos.x() <= 0 || pos.y() <= 0) return std::vector<Point3f*>();
//
//  const uint32_t HIT_TOLERANCE = 25;
//
//  std::vector<Point3f*> hittedPoints;
//  std::map<uint32_t, Point3f*> nameMap;
//
//  const double FOV = 45.0;
//
//  double ASPECT_RATIO = (double)width() / (double)height();
//  double MIN_Z = 0.1, MAX_Z = 327.68;
//  int viewport[4];
//  memset(viewport, 0, sizeof(viewport));
//  unsigned int buffer[256];
//  memset(buffer, 0, sizeof(buffer));
//
//  float lDepth[1];
//  lDepth[0] = 0;
//
//  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
//
//  glGetIntegerv(GL_VIEWPORT, viewport);
//
//  //      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//  glSelectBuffer(256, buffer);  // Den Puffer zuordnen
//
//  glMatrixMode(GL_PROJECTION);  // In den Projektionsmodus
//  glRenderMode(GL_SELECT);      // In den Selectionsmodus schalten
//  glPushMatrix();               // Um unsere Matrix zu sichern
//  glLoadIdentity();             // Und dieselbige wieder zurueckzusetzen
//
//  gluPickMatrix(pos.x(), viewport[3] - pos.y(), 1.0, 1.0, viewport);
//  gluPerspective(FOV, ASPECT_RATIO, MIN_Z, MAX_Z);
//
//  glInitNames();
//  glPushName(0);
//
//  glMatrixMode(GL_MODELVIEW);
//  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//  glDisable(GL_CULL_FACE);
//
//  glPushMatrix();
//  /** important: the conversion from RoSe to OpenGL coordinates. **/
//  glMultMatrixf(rv::RoSe2GL);
//
//  /** draw bounding boxes of segments. **/
//  int32_t name = 1;
//  for (uint32_t i = 0; i < cylinders->size(); ++i) {
//    if (&(*cylinders)[i] == newCylinder) continue;
//
//    bool found = false;
//
//    for (uint32_t j = 0; j < mFilteredLabels.size(); ++j) {
//      if (mFilteredLabels[j] == (*cylinders)[i].label) {
//        found = true;
//        break;
//      }
//    }
//
//    if (!found && mFilteredLabels.size() > 0) continue;
//
//    bool sfound = false;
//    bool efound = false;
//
//    Point3f& s = (*cylinders)[i].s;
//    Point3f& e = (*cylinders)[i].e;
//
//    for (uint32_t j = 0; j < selectedCylinderPoints.size(); ++j) {
//      if (selectedCylinderPoints[j] == &s) sfound = true;
//      if (selectedCylinderPoints[j] == &e) efound = true;
//    }
//
//    nameMap[name] = &s;
//    nameMap[name + 1] = &e;
//    if (!sfound) {
//      glLoadName(name);
//      glPushMatrix();
//      glTranslatef(s.x, s.y, s.z);
//      drawSphere(mCylinderPointSize);
//      glPopMatrix();
//    }
//
//    if (!efound) {
//      glLoadName(name + 1);
//      glPushMatrix();
//      glTranslatef(e.x, e.y, e.z);
//      drawSphere(mCylinderPointSize);
//      glPopMatrix();
//    }
//    name += 2;
//  }
//
//  glPopMatrix();
//
//  glMatrixMode(GL_PROJECTION);  // Wieder in den Projektionsmodus
//  glPopMatrix();                // und unsere alte Matrix wiederherzustellen
//
//  glMatrixMode(GL_MODELVIEW);
//  int hits = glRenderMode(GL_RENDER);  // Anzahl der Treffer auslesen
//
//  glPushMatrix();
//  /** important: the conversion from RoSe to OpenGL coordinates. **/
//  glMultMatrixf(rv::RoSe2GL);
//
//  name = 1;
//  for (uint32_t i = 0; i < cylinders->size(); ++i) {
//    if (&(*cylinders)[i] == newCylinder) continue;
//
//    bool found = false;
//
//    for (uint32_t j = 0; j < mFilteredLabels.size(); ++j) {
//      if (mFilteredLabels[j] == (*cylinders)[i].label) {
//        found = true;
//        break;
//      }
//    }
//
//    if (!found && mFilteredLabels.size() > 0) continue;
//
//    Point3f& s = (*cylinders)[i].s;
//    Point3f& e = (*cylinders)[i].e;
//
//    bool sfound = false, efound = false;
//    for (uint32_t j = 0; j < selectedCylinderPoints.size(); ++j) {
//      if (selectedCylinderPoints[j] == &s) sfound = true;
//      if (selectedCylinderPoints[j] == &e) efound = true;
//    }
//
//    if (!sfound) {
//      glLoadName(name);
//      glPushMatrix();
//      glTranslatef(s.x, s.y, s.z);
//      drawSphere(mCylinderPointSize);
//      glPopMatrix();
//    }
//
//    if (!efound) {
//      glLoadName(name + 1);
//      glPushMatrix();
//      glTranslatef(e.x, e.y, e.z);
//      drawSphere(mCylinderPointSize);
//      glPopMatrix();
//    }
//    name += 2;
//  }
//  glPopMatrix();
//
//  glReadPixels(pos.x(), (viewport[3] - pos.y()), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, lDepth);
//
//  if (hits > 0) {
//    uint32_t minz = buffer[1];
//    uint32_t obj_name = buffer[3];
//    for (int32_t i = 0; i < hits; i++) {
//      if (buffer[(i * 4) + 1] < minz) {
//        obj_name = buffer[(i * 4) + 3];
//        minz = buffer[(i * 4) + 1];
//      }
//    }
//
//    if (obj_name > 0) {
//      for (int32_t i = 0; i < hits; i++) {
//        if (buffer[(i * 4) + 1] - minz < HIT_TOLERANCE) {
//          hittedPoints.push_back(nameMap[buffer[(i * 4) + 3]]);
//        }
//      }
//    }
//  }
//
//  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//  updateGL();
//
//  return hittedPoints;
//}

// bool Viewport::getClickedPoint(const QPoint& pos, Point3f& selectedPoint) {
//  /** todo: implement plane for xyz **/
//  if (mAxis == XYZ) return false;
//  if (pos.x() >= width() || pos.y() >= height() || pos.x() <= 0 || pos.y() <= 0) return false;
//
//  bool pointSelected = false;
//  Point3f origin;
//
//  if (newCylinder != 0) {
//    origin = newCylinder->s;
//  } else if (selectedCylinderPoints.size() > 0) {
//    origin = *selectedCylinderPoints[0];
//  }
//
//  float planeExtent = 100000.0f;
//  Point3f p1, p2, p3, p4;
//  if (mAxis == X) {
//    p1 = Point3f(0.0f, -planeExtent, planeExtent);
//    p2 = Point3f(0.0f, -planeExtent, -planeExtent);
//    p3 = Point3f(0.0f, planeExtent, -planeExtent);
//    p4 = Point3f(0.0f, planeExtent, planeExtent);
//  } else if (mAxis == Y) {
//    p1 = Point3f(planeExtent, 0.0f, planeExtent);
//    p2 = Point3f(planeExtent, 0.0f, -planeExtent);
//    p3 = Point3f(-planeExtent, 0.0f, -planeExtent);
//    p4 = Point3f(-planeExtent, 0.0f, planeExtent);
//  } else if (mAxis == Z) {
//    p1 = Point3f(planeExtent, planeExtent, 0.0f);
//    p2 = Point3f(-planeExtent, planeExtent, 0.0f);
//    p3 = Point3f(-planeExtent, -planeExtent, 0.0f);
//    p4 = Point3f(planeExtent, -planeExtent, 0.0f);
//  }
//
//  for (uint32_t i = 0; i < 3; ++i) {
//    p1[i] += origin[i];
//    p2[i] += origin[i];
//    p3[i] += origin[i];
//    p4[i] += origin[i];
//  }
//
//  std::vector<Point3f*> hittedPoints;
//  std::map<uint32_t, Point3f*> nameMap;
//
//  static const double FOV = 45.0;
//
//  const double ASPECT_RATIO = (double)width() / (double)height();
//  const double MIN_Z = 0.1, MAX_Z = 327.68;
//  int viewport[4];
//  memset(viewport, 0, sizeof(viewport));
//  unsigned int buffer[256];
//  memset(buffer, 0, sizeof(buffer));
//
//  float lDepth[1];
//  lDepth[0] = 0;
//
//  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
//
//  glGetIntegerv(GL_VIEWPORT, viewport);
//
//  //      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//  glSelectBuffer(256, buffer);  // Den Puffer zuordnen
//
//  glMatrixMode(GL_PROJECTION);  // In den Projektionsmodus
//  glRenderMode(GL_SELECT);      // In den Selectionsmodus schalten
//  glPushMatrix();               // Um unsere Matrix zu sichern
//  glLoadIdentity();             // Und dieselbige wieder zurueckzusetzen
//
//  gluPickMatrix(pos.x(), viewport[3] - pos.y(), 1.0, 1.0, viewport);
//  gluPerspective(FOV, ASPECT_RATIO, MIN_Z, MAX_Z);
//
//  glInitNames();
//  glPushName(0);
//
//  glMatrixMode(GL_MODELVIEW);
//  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//  glDisable(GL_CULL_FACE);
//
//  glPushMatrix();
//  /** important: the conversion from RoSe to OpenGL coordinates. **/
//  glMultMatrixf(glow::RoSe2GL::matrix.data());
//
//  int32_t name = 1;
//  glLoadName(name);
//  glBegin(GL_QUADS);
//  glVertex3fv(&p1.x);
//  glVertex3fv(&p2.x);
//  glVertex3fv(&p3.x);
//  glVertex3fv(&p4.x);
//  glEnd();
//
//  glPopMatrix();
//
//  glMatrixMode(GL_PROJECTION);  // Wieder in den Projektionsmodus
//  glPopMatrix();                // und unsere alte Matrix wiederherzustellen
//
//  glMatrixMode(GL_MODELVIEW);
//  int hits = glRenderMode(GL_RENDER);  // Anzahl der Treffer auslesen
//
//  glPushMatrix();
//  /** important: the conversion from RoSe to OpenGL coordinates. **/
//  glMultMatrixf(glow::RoSe2GL::matrix.data());
//
//  name = 1;
//  glLoadName(name);
//  glBegin(GL_QUADS);
//  glVertex3fv(&p1.x);
//  glVertex3fv(&p2.x);
//  glVertex3fv(&p3.x);
//  glVertex3fv(&p4.x);
//  glEnd();
//  glPopMatrix();
//
//  glReadPixels(pos.x(), (viewport[3] - pos.y()), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, lDepth);
//
//  if (hits > 0) {
//    uint32_t minz = buffer[1];
//    uint32_t obj_name = buffer[3];
//    for (int32_t i = 0; i < hits; i++) {
//      if (buffer[(i * 4) + 1] < minz) {
//        obj_name = buffer[(i * 4) + 3];
//        minz = buffer[(i * 4) + 1];
//      }
//    }
//
//    if (obj_name == 1) {
//      // calculate the unprojected point only if the ground has been hit
//      double modMatrix[16];
//      double projMatrix[16];
//
//      glGetDoublev(GL_MODELVIEW_MATRIX, modMatrix);
//      glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
//      double p[3];
//      gluUnProject(pos.x(), (viewport[3] - pos.y()), lDepth[0], modMatrix, projMatrix, viewport, &p[0], &p[1],
//      &p[2]);
//
//      /** change selected point to RoSe convention **/
//      selectedPoint[0] = -p[2];
//      selectedPoint[2] = p[1];
//      selectedPoint[1] = -p[0];
//      pointSelected = true;
//    }
//  }
//
//  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//  updateGL();
//
//  return pointSelected;
//}

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
