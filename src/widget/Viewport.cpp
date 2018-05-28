#include "Viewport.h"
#include "../data/ColorGL.h"
#include "../data/CoordinateSystems.h"
#include "../data/Math.h"
#include "../data/draw_utils.h"
#include "../data/transform.h"

Viewport::Viewport(QWidget* parent, Qt::WindowFlags f) :
  QGLWidget(parent, 0, f), points(0), labels(0), cylinders(0),
      mHeadingStart(0.0), mTiltStart(0.0), mSlideMode(false),
      mChangeCamera(false), mAxis(XYZ), mMode(NONE), mFlags(FLAG_OVERWRITE),
      mPointSize(1.0f), mCylinderPointSize(0.5f), mCurrentLabel(0), mRadius(5),
      selectedCylinder(0), newCylinder(0), startPointSet(false),
      endPointSet(false), pointInitialized(false), buttonPressed(false)
{
  connect(&mMoveTimer, SIGNAL(timeout()), this, SLOT(moveCamera()),
      Qt::DirectConnection);
  mCamera.setXYZ(-50, 50, -50);
  mCamera.setYaw(Math::deg2rad(-125.0));
  mCamera.setPitch(Math::deg2rad(-45.0));

  setMouseTracking(true);
}

Viewport::~Viewport()
{

}

/** \brief set axis fixed (x = 1, y = 2, z = 3) **/
void Viewport::setFixedAxis(AXIS axis)
{
  mAxis = axis;
  if (axis == X)
  {
    mCamera.setXYZ(0, 0, -50);
    mCamera.setYaw(Math::deg2rad(-180));
    mCamera.setPitch(0);
  }
  else if (axis == Y)
  {
    mCamera.setXYZ(-50, 0, 0);
    mCamera.setYaw(Math::deg2rad(-90.0f));
    mCamera.setPitch(0);
  }
  else if (axis == Z)
  {
    mCamera.setXYZ(0, 50, 0);
    mCamera.setYaw(0);
    mCamera.setPitch(Math::deg2rad(-90.0f));
  }
}

void Viewport::setPoints(const std::vector<Point3f>& p,
    std::vector<uint32_t>& l)
{
  points = &p;
  labels = &l;
  updateGL();
}

void Viewport::setCylinders(std::vector<Cylinder>& c)
{
  cylinders = &c;
  updateGL();
}

void Viewport::setSelectedCylinder(Cylinder* cylinder, bool isNew)
{
  newCylinder = 0;
  selectedCylinder = cylinder;
  selectedCylinderPoints.clear();
  startPointSet = false;
  endPointSet = false;
  pointInitialized = false;

  if (isNew) newCylinder = cylinder;

  updateGL();
}

void Viewport::setRadius(float value)
{
  mRadius = value;
}

void Viewport::setLabel(uint32_t label)
{
  mCurrentLabel = label;
}

void Viewport::setLabelColors(const std::map<uint32_t, ColorGL>& colors)
{
  mLabelColors = colors;
}

void Viewport::setPointSize(int value)
{
  mPointSize = value;
  updateGL();
}

void Viewport::setMode(MODE mode)
{
  mMode = mode;
  updateGL();
}

void Viewport::setFlags(int32_t flags)
{
  mFlags = flags;
}

void Viewport::setOverwrite(bool value)
{
  if (value)
    mFlags = mFlags | FLAG_OVERWRITE;
  else
    mFlags = mFlags & ~FLAG_OVERWRITE;
}

void Viewport::setFilteredLabels(const std::vector<uint32_t>& labels)
{
  mFilteredLabels = labels;
  updateGL();
}

void Viewport::moveCamera()
{
  QTime tick = QTime::currentTime();
  if (tick > mLastTick)
  {
    float timeInSecs = (float(mLastTick.msecsTo(tick)) / 1000.0f);
    mCamera.applyVelocity(timeInSecs);
    updateGL();
    mLastTick = tick;
  }
}

void Viewport::initializeGL()
{
  static const float DIFFUSE[4] =
  { 1.00f, 1.00f, 1.00f, 1.00f };
  static const float AMBIENT[4] =
  { 0.00f, 0.00f, 0.00f, 1.00f };
  static const float SPECULAR[4] =
  { 0.00f, 0.00f, 0.00f, 1.00f };

  glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, DIFFUSE);
  glLightfv(GL_LIGHT0, GL_AMBIENT, AMBIENT);
  glLightfv(GL_LIGHT0, GL_SPECULAR, SPECULAR);
  glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 1.0);
  glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.002); // (4 / maxdist)

  //  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LIGHT0);
  glDisable(GL_LIGHTING);

  glShadeModel(GL_SMOOTH);
  glColorMaterial(GL_FRONT, GL_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);

  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_POINT_SMOOTH);
  //  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

  glEnable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_POINT_SMOOTH);
  //  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
}

void Viewport::resizeGL(int width, int height)
{
  const double FOV = 45.0;
  double ASPECT_RATIO = double(width) / double(height);
  const double MIN_Z = 1.0, MAX_Z = 327.68;
  const float LIGHT_POS[] =
  { 0.0f, 1.0f, 0.0f, 1.0f };

  glViewport(0, 0, width, height);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glLightfv(GL_LIGHT0, GL_POSITION, LIGHT_POS);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(FOV, ASPECT_RATIO, MIN_Z, MAX_Z);
}

void Viewport::paintGL()
{

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  mCamera.applyGL();

  vf.update();

  /** update the projected coordinates. **/
  updateProjections();

  glPushMatrix();

  glMultMatrixf(rv::RoSe2GL);

  glBegin(GL_LINES);
  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(5.0f, 0.0f, 0.0f);
  glColor3f(0.0f, 1.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 5.0f, 0.0f);
  glColor3f(0.0f, 0.0f, 1.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 5.0f);
  glEnd();

  glPopMatrix();

  if (points != 0 && labels != 0) drawPoints(*points, *labels);
  if (cylinders != 0 && mMode == CYLINDER) drawCylinders(*cylinders);
}

void Viewport::mousePressEvent(QMouseEvent* e)
{
  mChangeCamera = false;

  if (e->modifiers() == Qt::ControlModifier || (mMode != PAINT && mMode
      != CYLINDER))
  //|| !((mCurrentPaintMode & PAINT_BRUSH) || (mCurrentPaintMode & PAINT_FILLPOLYGON) || mSelectionMode))
  {
    mMouseStart = e->pos();
    mHeadingStart = mCamera.yaw();
    mTiltStart = mCamera.pitch();
    mSlideMode = e->modifiers() & Qt::ShiftModifier;
    mLastTick = QTime::currentTime();

    mChangeCamera = true;
  }
  else if (mMode == CYLINDER && mAxis != XYZ)
  {

    /** first check if cylinder point selected **/
    std::vector<Point3f*> selectedPoints = getSelectedCylinderPoints(e->pos());

    if (newCylinder != 0 && !newCylinder->startPointInitialized)
    {
      newCylinder->startPointInitialized = true;
      selectedCylinderPoints.push_back(&newCylinder->s);
    }
    else if (newCylinder != 0 && !newCylinder->endPointInitialized)
    {
      newCylinder->endPointInitialized = true;
      selectedCylinderPoints.push_back(&newCylinder->e);
    }

    if (selectedPoints.size() == 0 && newCylinder != 0)
    {
      std::cout << "setting cylinder point to new coordinate." << std::endl;
      /** set corresponding new cylinder point **/
      Point3f point;
      if (getClickedPoint(e->pos(), point))
      {
        *selectedCylinderPoints[0] = point;
        pointInitialized = true;
        emit cylinderChanged();
      }
    }
    else if (selectedPoints.size() > 0 && newCylinder != 0)
    {
      std::cout << "assigning cylinder point to other coordinate." << std::endl;
      *selectedCylinderPoints[0] = *selectedPoints[0];
      pointInitialized = true;
      emit cylinderChanged();
    }
    else if (selectedPoints.size() > 0)
    {
      selectedCylinderPoints = selectedPoints;
    }
    else
      selectedCylinderPoints.clear();

    updateGL();
  }
  else if (mMode == PAINT)
  {
    buttonPressed = true;
    if (e->buttons() & Qt::LeftButton)
      labelPoints(e->x(), e->y(), mRadius, mCurrentLabel);
    else if (e->buttons() & Qt::RightButton) labelPoints(e->x(), e->y(),
        mRadius, 0);

    updateGL();
  }
  //  else if ((mCurrentPaintMode & PAINT_FILLPOLYGON) && (e->buttons() & Qt::LeftButton))
  //  {
  //    vertices.push_back(RoSe::Vector2(e->x(), e->y()));
  //
  //    updateGL();
  //  }

  e->accept();
}

void Viewport::mouseReleaseEvent(QMouseEvent* e)
{
  if (mChangeCamera)
  {
    mCamera.setVelocity(0.0f, 0.0f, 0.0f, 0.0f);
    if (mMoveTimer.isActive()) mMoveTimer.stop();
  }
  else if (mMode == CYLINDER && mAxis != XYZ)
  {
    if (newCylinder != 0)
    {
      Point3f point;
      std::vector<Point3f*> selectedPoints =
          getSelectedCylinderPoints(e->pos());

      if (selectedPoints.size() > 0)
      {
        *selectedCylinderPoints[0] = *selectedPoints[0];
      }
      else if (getClickedPoint(e->pos(), point))
      {
        *selectedCylinderPoints[0] = point;
      }

      pointInitialized = false;

      if (newCylinder->startPointInitialized
          && newCylinder->endPointInitialized)
      {
        newCylinder = 0;
        selectedCylinder = 0;
        selectedCylinderPoints.clear();

        emit newCylinderFinished();
      }

      emit cylinderChanged();
    }
    else if (selectedCylinderPoints.size() > 0)
    {
      Point3f point;
      std::vector<Point3f*> selectedPoints =
          getSelectedCylinderPoints(e->pos());

      if (selectedPoints.size() > 0)
      {
        *selectedCylinderPoints[0] = *selectedPoints[0];
      }
      else if (getClickedPoint(e->pos(), point))
      {
        *selectedCylinderPoints[0] = point;
      }

      emit cylinderChanged();
    }

    selectedCylinderPoints.clear();
  }
  else if (mMode == PAINT)
  {
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
  //    /** (1) determine winding order, really bullet proof is this method not, but it should work in most situations. **/
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
  e->accept();
}

void Viewport::mouseMoveEvent(QMouseEvent* e)
{
  static const int MIN_MOVE = 20;
  static const float WALK_SENSITIVITY = -0.1f;
  static const float TURN_SENSITIVITY = -0.01f;
  static const float SLIDE_SENSITIVITY = 0.1f;
  static const float RAISE_SENSITIVITY = -0.1f;
  static const float LOOK_SENSITIVITY = -0.01f;
  static const float FREE_TURN_SENSITIVITY = -0.01f;
  if (mChangeCamera)
  {
    int dx = e->pos().x() - mMouseStart.x();
    int dy = e->pos().y() - mMouseStart.y();
    bool changedView = false;
    if (abs(dx) < MIN_MOVE)
    {
      dx = 0;
    }
    else
    {
      if (dx > 0)
        dx -= MIN_MOVE;
      else
        dx += MIN_MOVE;
    }
    if (abs(dy) < MIN_MOVE)
    {
      dy = 0;
    }
    else
    {
      if (dy > 0)
        dy -= MIN_MOVE;
      else
        dy += MIN_MOVE;
    }
    if (e->buttons() & Qt::LeftButton)
    {
      mCamera.setVelocity(WALK_SENSITIVITY * dy, 0.0f, 0.0f,
          (mAxis == XYZ) ? (TURN_SENSITIVITY * dx) : 0.0f);
    }
    else if (e->buttons() & Qt::MidButton)
    {
      mCamera.setVelocity(0.0f, RAISE_SENSITIVITY * dy, SLIDE_SENSITIVITY * dx,
          0.0f);
    }
    else if (e->buttons() & Qt::RightButton)
    {
      if (mAxis == XYZ)
      {
        mCamera.setVelocity(0.0f, 0.0f, 0.0f, 0.0f);
        float turn = FREE_TURN_SENSITIVITY * (e->pos().x() - mMouseStart.x());
        float tilt = LOOK_SENSITIVITY * (e->pos().y() - mMouseStart.y());
        mCamera.setYaw(mHeadingStart + turn);
        mCamera.setPitch(mTiltStart + tilt);
        changedView = true;
      }
    }
    else
    {
      mCamera.setVelocity(0.0f, 0.0f, 0.0f, 0.0f);
    }

    if (mCamera.hasVelocity())
    {
      if (!mMoveTimer.isActive()) mMoveTimer.start();
    }
    else
    {
      if (mMoveTimer.isActive()) mMoveTimer.stop();
      if (changedView) updateGL();
    }
  }
  else if (mMode == PAINT)
  {
    if (buttonPressed)
    {
      if (e->buttons() & Qt::LeftButton)
        labelPoints(e->x(), e->y(), mRadius, mCurrentLabel);
      else
        labelPoints(e->x(), e->y(), mRadius, 0);
    }
    updateGL();
  }
  else if (mMode == CYLINDER && mAxis != XYZ)
  {
    if (selectedCylinderPoints.size() > 0 && newCylinder != 0)
    {
      Point3f point;
      std::vector<Point3f*> selectedPoints =
          getSelectedCylinderPoints(e->pos());

      if (selectedPoints.size() > 0)
      {
        *selectedCylinderPoints[0] = *selectedPoints[0];
        emit cylinderChanged();
      }
      else if (getClickedPoint(e->pos(), point))
      {
        *selectedCylinderPoints[0] = point;
        emit cylinderChanged();
      }
    }
    else if (selectedCylinderPoints.size() > 0)
    {
      Point3f point;
      if (getClickedPoint(e->pos(), point))
      {
        for (uint32_t i = 0; i < selectedCylinderPoints.size(); ++i)
          *selectedCylinderPoints[i] = point;
        emit cylinderChanged();
      }
    }

    hoverCylinderPoints = getSelectedCylinderPoints(e->pos());

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
  e->accept();
}

void Viewport::keyPressEvent(QKeyEvent*)
{

}

void Viewport::drawPoints(const std::vector<Point3f>& points,
    const std::vector<uint32_t>& labels)
{
  glPushMatrix();
  glMultMatrixf(rv::RoSe2GL);

  glPointSize(mPointSize);
  glColor3fv(ColorGL::BLACK);

  glBegin(GL_POINTS);
  for (uint32_t i = 0; i < points.size(); ++i)
  {

    bool found = false;

    for (uint32_t j = 0; j < mFilteredLabels.size(); ++j)
    {
      if (mFilteredLabels[j] == labels[i])
      {
        found = true;
        break;
      }
    }

    if (!found && mFilteredLabels.size() > 0) continue;

    if (mLabelColors.find(labels[i]) == mLabelColors.end())
      glColor3fv(ColorGL::BLACK);
    else
      glColor3fv(mLabelColors[labels[i]]);
    glVertex3fv(&points[i].x);
  }
  glEnd();

  glPopMatrix();
}

void Viewport::drawCylinders(const std::vector<Cylinder>& cylinders)
{
  glPushMatrix();
  glMultMatrixf(rv::RoSe2GL);
  glCullFace(GL_BACK);
  glPolygonMode(GL_FRONT, GL_FILL);

  for (uint32_t i = 0; i < cylinders.size(); ++i)
  {
    if (&cylinders[i] == newCylinder)
    {
      const Point3f& s = cylinders[i].s;
      const Point3f& e = cylinders[i].e;

      if (newCylinder->startPointInitialized
          && !newCylinder->endPointInitialized)
      {
        glColor3fv(ColorGL::RED);
        glPushMatrix();
        glTranslatef(s.x, s.y, s.z);
        drawSphere(mCylinderPointSize);
        glPopMatrix();
      }
      else
      {
        glColor3fv(ColorGL::GOLD);
        glPushMatrix();
        glTranslatef(s.x, s.y, s.z);
        drawSphere(mCylinderPointSize);
        glPopMatrix();

        if (newCylinder->endPointInitialized)
        {
          glColor3fv(ColorGL::RED);
          glPushMatrix();
          glTranslatef(e.x, e.y, e.z);
          drawSphere(mCylinderPointSize);
          glPopMatrix();

          float length = (e - s).Length();
          if (length > 0.001)
          {
            Transform t;
            Vector3f xAxis(1.0f, 0.0f, 0.0f);
            Vector3f yAxis(0.0f, 1.0f, 1.0f);

            Vector3f z = e - s;
            z = Normalize(z);
            Vector3f y = Cross(z, xAxis);
            if (y.Length() < 0.0001) y = Cross(z, yAxis);
            y = Normalize(y);
            Vector3f x = Cross(z, y);
            x = Normalize(x);

            Matrix4x4 m;
            m(0, 0) = x.x;
            m(1, 0) = x.y;
            m(2, 0) = x.z;
            m(0, 1) = y.x;
            m(1, 1) = y.y;
            m(2, 1) = y.z;
            m(0, 2) = z.x;
            m(1, 2) = z.y;
            m(2, 2) = z.z;

            glColor4f(ColorGL::BLACK.R, ColorGL::BLACK.G, ColorGL::BLACK.B,
                0.5f);
            glPushMatrix();
            glTranslatef(s.x, s.y, s.z);
            glMultMatrixf(&m.m[0]);
            drawCylinder(cylinders[i].radius, length);
            glPopMatrix();
          }
        }
      }
    }
    else if (&cylinders[i] == selectedCylinder)
    {
      const Point3f& s = cylinders[i].s;
      const Point3f& e = cylinders[i].e;

      glColor3fv(ColorGL::GOLD);
      for (uint32_t j = 0; j < hoverCylinderPoints.size(); ++j)
      {
        if (hoverCylinderPoints[j] == &s)
        {
          glColor3fv(ColorGL::GREEN);
          break;
        }
      }

      if (cylinders[i].startPointInitialized)
      {
        glPushMatrix();
        glTranslatef(s.x, s.y, s.z);
        drawSphere(mCylinderPointSize);
        glPopMatrix();
      }

      glColor3fv(ColorGL::GOLD);
      for (uint32_t j = 0; j < hoverCylinderPoints.size(); ++j)
      {
        if (hoverCylinderPoints[j] == &e)
        {
          glColor3fv(ColorGL::GREEN);
          break;
        }
      }

      if (cylinders[i].startPointInitialized
          && cylinders[i].endPointInitialized)
      {
        glPushMatrix();
        glTranslatef(e.x, e.y, e.z);
        drawSphere(mCylinderPointSize);
        glPopMatrix();

        float length = (e - s).Length();
        if (length > 0.001)
        {
          Transform t;
          Vector3f xAxis(1.0f, 0.0f, 0.0f);
          Vector3f yAxis(0.0f, 1.0f, 1.0f);

          Vector3f z = e - s;
          z = Normalize(z);
          Vector3f y = Cross(z, xAxis);
          if (y.Length() < 0.0001) y = Cross(z, yAxis);
          y = Normalize(y);
          Vector3f x = Cross(z, y);
          x = Normalize(x);

          Matrix4x4 m;
          m(0, 0) = x.x;
          m(1, 0) = x.y;
          m(2, 0) = x.z;
          m(0, 1) = y.x;
          m(1, 1) = y.y;
          m(2, 1) = y.z;
          m(0, 2) = z.x;
          m(1, 2) = z.y;
          m(2, 2) = z.z;

          if (cylinders[i].label > 0 && mLabelColors.find(cylinders[i].label)
              != mLabelColors.end())
          {
            const ColorGL color = mLabelColors[cylinders[i].label];
            glColor4f(color.R, color.G, color.B, 0.5f);
          }
          else
            glColor4f(ColorGL::GOLD.R, ColorGL::GOLD.G, ColorGL::GOLD.B, 0.5f);

          glPushMatrix();
          glTranslatef(s.x, s.y, s.z);
          glMultMatrixf(&m.m[0]);
          drawCylinder(cylinders[i].radius, length);
          glPopMatrix();
        }
      }
    }
    else
    {
      bool found = false;

      for (uint32_t j = 0; j < mFilteredLabels.size(); ++j)
      {
        if (mFilteredLabels[j] == cylinders[i].label)
        {
          found = true;
          break;
        }
      }

      if (!found && mFilteredLabels.size() > 0) continue;

      const Point3f& s = cylinders[i].s;
      const Point3f& e = cylinders[i].e;

      glColor3fv(ColorGL::BLACK);
      for (uint32_t j = 0; j < hoverCylinderPoints.size(); ++j)
      {
        if (hoverCylinderPoints[j] == &s)
        {
          glColor3fv(ColorGL::GREEN);
          break;
        }
      }
      glPushMatrix();
      glTranslatef(s.x, s.y, s.z);
      drawSphere(mCylinderPointSize);
      glPopMatrix();

      glColor3fv(ColorGL::BLACK);
      for (uint32_t j = 0; j < hoverCylinderPoints.size(); ++j)
      {
        if (hoverCylinderPoints[j] == &e)
        {
          glColor3fv(ColorGL::GREEN);
          break;
        }
      }
      glPushMatrix();
      glTranslatef(e.x, e.y, e.z);
      drawSphere(mCylinderPointSize);
      glPopMatrix();

      float length = (e - s).Length();
      if (length > 0.001)
      {
        Transform t;
        Vector3f xAxis(1.0f, 0.0f, 0.0f);
        Vector3f yAxis(0.0f, 1.0f, 1.0f);

        Vector3f z = e - s;
        z = Normalize(z);
        Vector3f y = Cross(z, xAxis);
        if (y.Length() < 0.0001) y = Cross(z, yAxis);
        y = Normalize(y);
        Vector3f x = Cross(z, y);
        x = Normalize(x);

        Matrix4x4 m;
        m(0, 0) = x.x;
        m(1, 0) = x.y;
        m(2, 0) = x.z;
        m(0, 1) = y.x;
        m(1, 1) = y.y;
        m(2, 1) = y.z;
        m(0, 2) = z.x;
        m(1, 2) = z.y;
        m(2, 2) = z.z;

        if (mLabelColors.find(cylinders[i].label) != mLabelColors.end())
        {
          const ColorGL color = mLabelColors[cylinders[i].label];
          glColor4f(color.R, color.G, color.B, 0.5f);
        }
        else
          glColor4f(ColorGL::BLACK.R, ColorGL::BLACK.G, ColorGL::BLACK.B, 0.5f);

        glPushMatrix();
        glTranslatef(s.x, s.y, s.z);
        glMultMatrixf(&m.m[0]);
        drawCylinder(cylinders[i].radius, length);
        glPopMatrix();
      }
    }
  }

  glPopMatrix();
}

void Viewport::labelPoints(int32_t x, int32_t y, float radius,
    uint32_t new_label)
{
  if (points == 0 || labels == 0) return;

  float radius2 = radius * radius;

  //  float min_distance2 = min_distance * min_distance;
  //  float max_distance2 = max_distance * max_distance;

  for (uint32_t i = 0; i < points->size(); ++i)
  {
    //    Point3f& p = scan[i].pos;
    //    Point3f global_pos = scan.pose()(scan[i].pos);
    //    float distance = p.x * p.x + p.y * p.y + p.z * p.z;
    //
    //    Label label = labels[i];
    //    if (do_mapping && mapping.find(label) != mapping.end()) label = mapping[label];
    //    if (do_filtering && filter.find(label) == filter.end()) continue;
    //
    //    if (mFilterPointsDistance && (distance > max_distance2 || distance < min_distance2)) continue;
    //    if (!mShowLabeledPoints && label != 0) continue;

    if (!vf.isInside((*points)[i].x, (*points)[i].y, (*points)[i].z)) continue;

    bool found = false;

    for (uint32_t j = 0; j < mFilteredLabels.size(); ++j)
      if (mFilteredLabels[j] == (*labels)[i])
      {
        found = true;
        break;
      }

    if (!found && mFilteredLabels.size() > 0) continue;

    double dx = projected_points[i].x - x;
    double dy = projected_points[i].y - y;
    if (dx * dx + dy * dy < radius2)
    {
      if (((*labels)[i] == 0) || (mFlags & FLAG_OVERWRITE))
      {
        (*labels)[i] = new_label;
      }
    }
  }

  emit labelingChanged();
}

void Viewport::updateProjections()
{
  if (points == 0 || labels == 0) return;

  glPushMatrix();
  glMultMatrixf(rv::RoSe2GL);

  GLint viewport[4];
  GLdouble modelMatrix[16];
  GLdouble projectionMatrix[16];

  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
  glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);

  projected_points.resize(points->size());
  for (uint32_t i = 0; i < points->size(); ++i)
  {
    double dummy = 0.0;

    const Point3f& p = (*points)[i];

    double x, y;

    gluProject(p.x, p.y, p.z, modelMatrix, projectionMatrix, viewport, &(x),
        &(y), &(dummy));
    projected_points[i].x = x;
    projected_points[i].y = y;
    /** the coordinate systems origin is in the top-left corner! **/
    projected_points[i].y = viewport[3] - projected_points[i].y;
  }

  glPopMatrix();
}

std::vector<Point3f*> Viewport::getSelectedCylinderPoints(const QPoint& pos)
{
  if (cylinders == 0) return std::vector<Point3f*>();
  if (mAxis == XYZ) return std::vector<Point3f*>();
  if (pos.x() >= width() || pos.y() >= height() || pos.x() <= 0 || pos.y() <= 0) return std::vector<
      Point3f*>();

  const uint32_t HIT_TOLERANCE = 25;

  std::vector<Point3f*> hittedPoints;
  std::map<uint32_t, Point3f*> nameMap;

  const double FOV = 45.0;

  double ASPECT_RATIO = (double) width() / (double) height();
  double MIN_Z = 0.1, MAX_Z = 327.68;
  int viewport[4];
  memset(viewport, 0, sizeof(viewport));
  unsigned int buffer[256];
  memset(buffer, 0, sizeof(buffer));

  float lDepth[1];
  lDepth[0] = 0;

  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  glGetIntegerv(GL_VIEWPORT, viewport);

  //      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glSelectBuffer(256, buffer); // Den Puffer zuordnen

  glMatrixMode(GL_PROJECTION); // In den Projektionsmodus
  glRenderMode(GL_SELECT); // In den Selectionsmodus schalten
  glPushMatrix(); // Um unsere Matrix zu sichern
  glLoadIdentity(); // Und dieselbige wieder zurueckzusetzen

  gluPickMatrix(pos.x(), viewport[3] - pos.y(), 1.0, 1.0, viewport);
  gluPerspective(FOV, ASPECT_RATIO, MIN_Z, MAX_Z);

  glInitNames();
  glPushName(0);

  glMatrixMode(GL_MODELVIEW);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glDisable(GL_CULL_FACE);

  glPushMatrix();
  /** important: the conversion from RoSe to OpenGL coordinates. **/
  glMultMatrixf(rv::RoSe2GL);

  /** draw bounding boxes of segments. **/
  int32_t name = 1;
  for (uint32_t i = 0; i < cylinders->size(); ++i)
  {
    if (&(*cylinders)[i] == newCylinder) continue;

    bool found = false;

    for (uint32_t j = 0; j < mFilteredLabels.size(); ++j)
    {
      if (mFilteredLabels[j] == (*cylinders)[i].label)
      {
        found = true;
        break;
      }
    }

    if (!found && mFilteredLabels.size() > 0) continue;

    bool sfound = false;
    bool efound = false;

    Point3f& s = (*cylinders)[i].s;
    Point3f& e = (*cylinders)[i].e;

    for (uint32_t j = 0; j < selectedCylinderPoints.size(); ++j)
    {
      if (selectedCylinderPoints[j] == &s) sfound = true;
      if (selectedCylinderPoints[j] == &e) efound = true;
    }

    nameMap[name] = &s;
    nameMap[name + 1] = &e;
    if (!sfound)
    {
      glLoadName(name);
      glPushMatrix();
      glTranslatef(s.x, s.y, s.z);
      drawSphere(mCylinderPointSize);
      glPopMatrix();
    }

    if (!efound)
    {
      glLoadName(name + 1);
      glPushMatrix();
      glTranslatef(e.x, e.y, e.z);
      drawSphere(mCylinderPointSize);
      glPopMatrix();
    }
    name += 2;
  }

  glPopMatrix();

  glMatrixMode(GL_PROJECTION); // Wieder in den Projektionsmodus
  glPopMatrix(); // und unsere alte Matrix wiederherzustellen

  glMatrixMode(GL_MODELVIEW);
  int hits = glRenderMode(GL_RENDER); // Anzahl der Treffer auslesen

  glPushMatrix();
  /** important: the conversion from RoSe to OpenGL coordinates. **/
  glMultMatrixf(rv::RoSe2GL);

  name = 1;
  for (uint32_t i = 0; i < cylinders->size(); ++i)
  {
    if (&(*cylinders)[i] == newCylinder) continue;

    bool found = false;

    for (uint32_t j = 0; j < mFilteredLabels.size(); ++j)
    {
      if (mFilteredLabels[j] == (*cylinders)[i].label)
      {
        found = true;
        break;
      }
    }

    if (!found && mFilteredLabels.size() > 0) continue;

    Point3f& s = (*cylinders)[i].s;
    Point3f& e = (*cylinders)[i].e;

    bool sfound = false, efound = false;
    for (uint32_t j = 0; j < selectedCylinderPoints.size(); ++j)
    {
      if (selectedCylinderPoints[j] == &s) sfound = true;
      if (selectedCylinderPoints[j] == &e) efound = true;
    }

    if (!sfound)
    {
      glLoadName(name);
      glPushMatrix();
      glTranslatef(s.x, s.y, s.z);
      drawSphere(mCylinderPointSize);
      glPopMatrix();
    }

    if (!efound)
    {
      glLoadName(name + 1);
      glPushMatrix();
      glTranslatef(e.x, e.y, e.z);
      drawSphere(mCylinderPointSize);
      glPopMatrix();
    }
    name += 2;
  }
  glPopMatrix();

  glReadPixels(pos.x(), (viewport[3] - pos.y()), 1, 1, GL_DEPTH_COMPONENT,
      GL_FLOAT, lDepth);

  if (hits > 0)
  {
    uint32_t minz = buffer[1];
    uint32_t obj_name = buffer[3];
    for (int32_t i = 0; i < hits; i++)
    {
      if (buffer[(i * 4) + 1] < minz)
      {
        obj_name = buffer[(i * 4) + 3];
        minz = buffer[(i * 4) + 1];
      }
    }

    if (obj_name > 0)
    {
      for (int32_t i = 0; i < hits; i++)
      {
        if (buffer[(i * 4) + 1] - minz < HIT_TOLERANCE)
        {
          hittedPoints.push_back(nameMap[buffer[(i * 4) + 3]]);
        }
      }
    }
  }

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  updateGL();

  return hittedPoints;
}

bool Viewport::getClickedPoint(const QPoint& pos, Point3f& selectedPoint)
{
  /** todo: implement plane for xyz **/
  if (mAxis == XYZ) return false;
  if (pos.x() >= width() || pos.y() >= height() || pos.x() <= 0 || pos.y() <= 0) return false;

  bool pointSelected = false;
  Point3f origin;

  if (newCylinder != 0)
  {
    origin = newCylinder->s;
  }
  else if (selectedCylinderPoints.size() > 0)
  {
    origin = *selectedCylinderPoints[0];
  }

  float planeExtent = 100000.0f;
  Point3f p1, p2, p3, p4;
  if (mAxis == X)
  {
    p1 = Point3f(0.0f, -planeExtent, planeExtent);
    p2 = Point3f(0.0f, -planeExtent, -planeExtent);
    p3 = Point3f(0.0f, planeExtent, -planeExtent);
    p4 = Point3f(0.0f, planeExtent, planeExtent);
  }
  else if (mAxis == Y)
  {
    p1 = Point3f(planeExtent, 0.0f, planeExtent);
    p2 = Point3f(planeExtent, 0.0f, -planeExtent);
    p3 = Point3f(-planeExtent, 0.0f, -planeExtent);
    p4 = Point3f(-planeExtent, 0.0f, planeExtent);
  }
  else if (mAxis == Z)
  {
    p1 = Point3f(planeExtent, planeExtent, 0.0f);
    p2 = Point3f(-planeExtent, planeExtent, 0.0f);
    p3 = Point3f(-planeExtent, -planeExtent, 0.0f);
    p4 = Point3f(planeExtent, -planeExtent, 0.0f);
  }

  for (uint32_t i = 0; i < 3; ++i)
  {
    p1[i] += origin[i];
    p2[i] += origin[i];
    p3[i] += origin[i];
    p4[i] += origin[i];
  }

  std::vector<Point3f*> hittedPoints;
  std::map<uint32_t, Point3f*> nameMap;

  static const double FOV = 45.0;

  const double ASPECT_RATIO = (double) width() / (double) height();
  const double MIN_Z = 0.1, MAX_Z = 327.68;
  int viewport[4];
  memset(viewport, 0, sizeof(viewport));
  unsigned int buffer[256];
  memset(buffer, 0, sizeof(buffer));

  float lDepth[1];
  lDepth[0] = 0;

  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  glGetIntegerv(GL_VIEWPORT, viewport);

  //      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glSelectBuffer(256, buffer); // Den Puffer zuordnen

  glMatrixMode(GL_PROJECTION); // In den Projektionsmodus
  glRenderMode(GL_SELECT); // In den Selectionsmodus schalten
  glPushMatrix(); // Um unsere Matrix zu sichern
  glLoadIdentity(); // Und dieselbige wieder zurueckzusetzen

  gluPickMatrix(pos.x(), viewport[3] - pos.y(), 1.0, 1.0, viewport);
  gluPerspective(FOV, ASPECT_RATIO, MIN_Z, MAX_Z);

  glInitNames();
  glPushName(0);

  glMatrixMode(GL_MODELVIEW);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glDisable(GL_CULL_FACE);

  glPushMatrix();
  /** important: the conversion from RoSe to OpenGL coordinates. **/
  glMultMatrixf(rv::RoSe2GL);

  int32_t name = 1;
  glLoadName(name);
  glBegin(GL_QUADS);
  glVertex3fv(&p1.x);
  glVertex3fv(&p2.x);
  glVertex3fv(&p3.x);
  glVertex3fv(&p4.x);
  glEnd();

  glPopMatrix();

  glMatrixMode(GL_PROJECTION); // Wieder in den Projektionsmodus
  glPopMatrix(); // und unsere alte Matrix wiederherzustellen

  glMatrixMode(GL_MODELVIEW);
  int hits = glRenderMode(GL_RENDER); // Anzahl der Treffer auslesen

  glPushMatrix();
  /** important: the conversion from RoSe to OpenGL coordinates. **/
  glMultMatrixf(rv::RoSe2GL);

  name = 1;
  glLoadName(name);
  glBegin(GL_QUADS);
  glVertex3fv(&p1.x);
  glVertex3fv(&p2.x);
  glVertex3fv(&p3.x);
  glVertex3fv(&p4.x);
  glEnd();
  glPopMatrix();

  glReadPixels(pos.x(), (viewport[3] - pos.y()), 1, 1, GL_DEPTH_COMPONENT,
      GL_FLOAT, lDepth);

  if (hits > 0)
  {
    uint32_t minz = buffer[1];
    uint32_t obj_name = buffer[3];
    for (int32_t i = 0; i < hits; i++)
    {
      if (buffer[(i * 4) + 1] < minz)
      {
        obj_name = buffer[(i * 4) + 3];
        minz = buffer[(i * 4) + 1];
      }
    }

    if (obj_name == 1)
    {
      // calculate the unprojected point only if the ground has been hit
      double modMatrix[16];
      double projMatrix[16];

      glGetDoublev(GL_MODELVIEW_MATRIX, modMatrix);
      glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
      double p[3];
      gluUnProject(pos.x(), (viewport[3] - pos.y()), lDepth[0], modMatrix,
          projMatrix, viewport, &p[0], &p[1], &p[2]);

      /** change selected point to RoSe convention **/
      selectedPoint[0] = -p[2];
      selectedPoint[2] = p[1];
      selectedPoint[1] = -p[0];
      pointSelected = true;
    }
  }

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  updateGL();

  return pointSelected;
}
