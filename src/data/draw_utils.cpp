#include "draw_utils.h"

#include <GL/gl.h>
#include <cstring>

#include "Math.h"
#include "ColorGL.h"
#include "transform.h"

void drawTriangles(uint32_t n, Point3f* vertices, int* indices)
{
  glBegin(GL_TRIANGLES);
  for (uint32_t i = 0; i < n; ++i)
  {
    /** TODO: normal computation **/
    //    Normal3f n = Normal3f(
    //        Cross(vertices[3 * i + 1] - vertices[3 * i],
    //            vertices[3 * i + 2] - vertices[3 * i]));
    //    n = Normalize(n);
    //    glNormal3fv(&n.x);
    glVertex3fv(&(vertices[indices[3 * i]].x));
    glVertex3fv(&(vertices[indices[3 * i + 1]].x));
    glVertex3fv(&(vertices[indices[3 * i + 2]].x));
  }
  glEnd();
  //  glLineWidth(4.0f);
  //  glColor3f(0.0f, 0.0f, 0.0f);
  //  glBegin(GL_LINES);
  //  for (uint32_t i = 0; i < n; ++i)
  //  {
  //    glVertex3fv(&(vertices[indices[3*i]].x));
  //    glVertex3fv(&(vertices[indices[3*i+1]].x));
  //
  //    glVertex3fv(&(vertices[indices[3*i+1]].x));
  //    glVertex3fv(&(vertices[indices[3*i+2]].x));
  //    glVertex3fv(&(vertices[indices[3*i+2]].x));
  //    glVertex3fv(&(vertices[indices[3*i]].x));
  //  }
  //  glEnd();
  //  glLineWidth(1.0f);
}

void drawCube(float sidelength)
{
  float s = sidelength / 2.0;

  glBegin(GL_QUAD_STRIP);
  glVertex3f(-s, -s, -s);//1
  glVertex3f(+s, -s, -s);//2
  glVertex3f(-s, -s, +s);//3
  glVertex3f(+s, -s, +s);//4
  glVertex3f(-s, +s, +s);//5
  glVertex3f(+s, +s, +s);//6
  glVertex3f(-s, +s, -s);//7
  glVertex3f(+s, +s, -s);//8
  glVertex3f(-s, -s, -s);//9
  glVertex3f(+s, -s, -s);//10
  glEnd();

  glBegin(GL_QUADS);
  glVertex3f(+s, -s, -s);//2
  glVertex3f(+s, +s, -s);//8
  glVertex3f(+s, +s, +s);//6
  glVertex3f(+s, -s, +s);//4
  glEnd();

  glBegin(GL_QUADS);
  glVertex3f(-s, -s, -s);//1
  glVertex3f(-s, +s, -s);//7
  glVertex3f(-s, +s, +s);//5
  glVertex3f(-s, -s, +s);//3
  glEnd();
}

void drawCylinder(float radius, float height, int n)
{
  // create vertices for bottom plane and top plane and transform them
  Point3f vertices[n * 2];

  float x = 0.0f;
  float y = 0.0f;

  for (int i = 0; i < n; ++i)
  {
    x = float(radius * std::cos(2.0f * i * Math::PI / n));
    y = float(radius * std::sin(2.0f * i * Math::PI / n));

    vertices[i] = Point3f(x, y, 0.0f);
    vertices[i + n] = Point3f(x, y, height);
  }

  /** triangle indices for the cylinder **/
  int numTriangles = 2 * n + 2 * (n - 2);
  int indices[3 * numTriangles];

  for (int i = 0, j = 0; i < n - 1; i++, j += 3)
  {
    indices[j + 0] = i;
    indices[j + 1] = i + 1;
    indices[j + 2] = i + 1 + n;

    indices[j + 0 + n * 3] = i + 1 + n;
    indices[j + 1 + n * 3] = i + n;
    indices[j + 2 + n * 3] = i;
  }

  indices[(n - 1) * 3] = n - 1;
  indices[(n - 1) * 3 + 1] = 0;
  indices[(n - 1) * 3 + 2] = n;
  indices[n * 3 * 2 - 3] = n;
  indices[n * 3 * 2 - 2] = 2 * n - 1;
  indices[n * 3 * 2 - 1] = n - 1;

  for (int i = 0, j = 0; i < n - 2; i++, j += 3)
  {
    indices[n * 3 * 2 + j + 0] = 0;
    indices[n * 3 * 2 + j + 1] = i + 2;
    indices[n * 3 * 2 + j + 2] = i + 1;

    indices[n * 3 * 2 + 3 * (n - 2) + j + 0] = n;
    indices[n * 3 * 2 + 3 * (n - 2) + j + 1] = n + i + 1;
    indices[n * 3 * 2 + 3 * (n - 2) + j + 2] = n + i + 2;
  }

  drawTriangles(numTriangles, vertices, indices);
}

void drawCone(float radius, float height, int n)
{

  // create vertices for bottom plane and top plane and transform them
  Point3f vertices[n + 1];
  float x = 0.0f;
  float y = 0.0f;

  for (int i = 0; i < n; ++i)
  {
    x = float(radius * std::cos(2 * i * Math::PI / n));
    y = float(radius * std::sin(2 * i * Math::PI / n));

    vertices[i] = Point3f(x, y, 0.0f);
  }

  Point3f tip(0.0f, 0.0f, height);
  vertices[n] = tip;
  int numTriangles = n + (n - 2);
  int indices[3 * numTriangles];

  for (int i = 0, j = 0; i < n - 1; ++i, j += 3)
  {
    indices[j + 0] = i;
    indices[j + 1] = i + 1;
    indices[j + 2] = n;
  }

  indices[(n - 1) * 3] = n - 1;
  indices[(n - 1) * 3 + 1] = 0;
  indices[(n - 1) * 3 + 2] = n;

  for (int i = 0, j = 0; i < n - 2; ++i, j += 3)
  {
    indices[n * 3 + j + 0] = 0;
    indices[n * 3 + j + 1] = i + 2;
    indices[n * 3 + j + 2] = i + 1;
  }

  drawTriangles(numTriangles, vertices, indices);
}

void drawSphere(float radius, int n)
{
  uint32_t numVertices = n * (n - 2) + 2;
  Point3f vertices[numVertices];

  float phi_increment = 2.0f * Math::PI / n;
  float theta_increment = Math::PI / (n - 1);

  /** using polar co-ordinates to generate vertices. **/
  float phi = 0.0f;
  for (int i = 0; i < n; ++i, phi += phi_increment)
  {
    float theta = theta_increment;
    for (int j = 1; j < n - 1; ++j, theta += theta_increment)
    {
      float x = radius * std::sin(theta) * std::cos(phi);
      float y = radius * std::sin(theta) * std::sin(phi);
      float z = radius * std::cos(theta);
      vertices[1 + i + (j - 1) * n] = Point3f(x, y, z);
    }
  }
  vertices[0] = Point3f(0, 0, radius);
  vertices[n * (n - 2) + 1] = Point3f(0, 0, -radius);

  int numTriangles = 2 * n * (n - 3) + 2 * n;
  int indices[3 * numTriangles];

  /** top part of the sphere **/
  int idx = 0, s = 1;
  for (int i = 0; i < n; ++i, idx += 3)
  {
    indices[idx] = s + i;
    indices[idx + 1] = (i < n - 1) ? (s + i + 1) : s;
    indices[idx + 2] = 0;
  }
  /** bottom part of the sphere **/
  s = n * (n - 3) + 1;
  for (int i = 0; i < n; ++i, idx += 3)
  {
    indices[idx] = (i < n - 1) ? (s + i + 1) : s;
    indices[idx + 1] = s + i;
    indices[idx + 2] = n * (n - 2) + 1;
  }

  /** levels in between **/
  for (int j = 1; j < n - 2; ++j)
  {
    s = (j - 1) * n + 1;
    for (int i = 0; i < n; ++i)
    {
      indices[idx] = s + i;
      indices[idx + 1] = s + i + n;
      indices[idx + 2] = (i < n - 1) ? (s + i + 1) : s;
      idx += 3;
      indices[idx] = s + n + i;
      indices[idx + 1] = (i < n - 1) ? (s + n + i + 1) : s + n;
      indices[idx + 2] = (i < n - 1) ? (s + i + 1) : s;
      idx += 3;
    }
  }
  assert(idx == 3 * numTriangles);

  drawTriangles(numTriangles, vertices, indices);
}
void drawArrow(const Point3f& start, const Vector3f& dir, float linewidth)
{
  Vector3f zAxis(0, 0, 1);
  Vector3f axis = Cross(zAxis, dir);
  float length = dir.Length();

  glPushMatrix();
  glTranslatef(start.x, start.y, start.z);

  if (axis.Length() > 0.001) /** encountered a parallel vector => no need to transform **/
  {
    Vector3f ndir = Normalize(dir);
    double angle = Math::rad2deg(acos(Dot(zAxis, ndir)));
    Transform t = Rotate(angle, axis);
    glMultMatrixf(t.GetMatrix().m);
  }

  drawCylinder(linewidth, length - 1.f);
  glTranslatef(0.0f, 0.0f, length - 1.f);
  drawCone(linewidth + 1.2f * linewidth, 1.f);
  glPopMatrix();
}

void drawArrow(const Point3f& start, const Point3f& end, float linewidth)
{
  Vector3f dir = end - start;
  drawArrow(start, dir, linewidth);
}
