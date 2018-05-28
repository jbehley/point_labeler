/* \brief a view, which can be used to label points.
 *
 *
 */
#ifndef POINTVIEW_H_
#define POINTVIEW_H_

#include <stdint.h>
#include <vector>
#include <list>

#include <QtOpenGL/QGLWidget>
#include <QtCore/QTimer>
#include <QtCore/QTime>
#include <QtGui/QMouseEvent>

#include <GL/gl.h>
#include <GL/glu.h>
#include "CameraGL.h"
#include "../data/geometry.h"
#include "../data/ColorGL.h"
#include "../data/Cylinder.h"
#include "../data/ViewFrustum.h"

class Viewport: public QGLWidget
{
  Q_OBJECT
  public:
    enum AXIS
    {
      XYZ, X, Y, Z
    };

    enum MODE
    {
      NONE, PAINT, CYLINDER
    };

    enum FLAGS
    {
      FLAG_OVERWRITE = 1, FLAG_OTHER = 2
    };

    Viewport(QWidget* parent = 0, Qt::WindowFlags f = 0);
    ~Viewport();

  signals:
    void newCylinderFinished();
    void cylinderChanged();
    void labelingChanged();

  public slots:
    /** \brief set axis fixed **/
    void setFixedAxis(AXIS axis);
    void setPoints(const std::vector<Point3f>& points,
        std::vector<uint32_t>& labels);
    void setCylinders(std::vector<Cylinder>& cylinders);

    void setSelectedCylinder(Cylinder* cylinder, bool isNew);

    void setRadius(float value);
    /** \brief label used when in PAINT **/
    void setLabel(uint32_t label);
    void setLabelColors(const std::map<uint32_t, ColorGL>& colors);
    void setPointSize(int value);

    void setMode(MODE mode);
    void setFlags(int32_t flags);
    void setOverwrite(bool value);

    void setFilteredLabels(const std::vector<uint32_t>& labels);

  protected slots:
    void moveCamera();

  protected:
    void initializeGL();
    void resizeGL(int width, int height);
    void paintGL();

    void mousePressEvent(QMouseEvent*);
    void mouseReleaseEvent(QMouseEvent*);
    void mouseMoveEvent(QMouseEvent*);
    void keyPressEvent(QKeyEvent*);

    void drawPoints(const std::vector<Point3f>& points,
        const std::vector<uint32_t>& labels);
    void drawCylinders(const std::vector<Cylinder>& cylinders);
    void labelPoints(int32_t x, int32_t y, float radius, uint32_t label);
    void updateProjections();

    std::vector<Point3f*>
        getSelectedCylinderPoints(const QPoint& mousePosition);
    /** \brief get 3d coordinates of a click.
     *
     *  \return true, if valid point has been clicked, false otherwise.
     */
    bool getClickedPoint(const QPoint& mousePosition, Point3f& clickedPoint);

    std::map<uint32_t, ColorGL> mLabelColors;

    const std::vector<Point3f>* points;
    std::vector<uint32_t>* labels;
    std::vector<Cylinder>* cylinders;
    std::vector<Point3f> projected_points;

    CameraGL mCamera;
    QTimer mMoveTimer;
    QTime mLastTick;
    float mHeadingStart, mTiltStart;
    bool mSlideMode, mChangeCamera, changedView;
    QPoint mMouseStart;

    AXIS mAxis;
    MODE mMode;
    int32_t mFlags;

    float mPointSize, mCylinderPointSize;
    uint32_t mCurrentLabel;
    float mRadius;
    std::vector<uint32_t> mFilteredLabels;

    /** selected endpoint **/
    std::vector<Point3f*> selectedCylinderPoints;
    Cylinder* selectedCylinder;
    Cylinder* newCylinder;
    bool startPointSet;
    bool endPointSet;
    bool pointInitialized;
    std::vector<Point3f*> hoverCylinderPoints;
    bool buttonPressed;
    ViewFrustum vf;
};

#endif /* POINTVIEW_H_ */
