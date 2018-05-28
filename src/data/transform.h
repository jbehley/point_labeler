/*
 pbrt source code Copyright(c) 1998-2010 Matt Pharr and Greg Humphreys.

 This file is part of pbrt.

 pbrt is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.  Note that the text contents of
 the book "Physically Based Rendering" are *not* licensed under the
 GNU GPL.

 pbrt is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 */

#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_CORE_TRANSFORM_H
#define PBRT_CORE_TRANSFORM_H

#include "geometry.h"
#include <cstdio>
#include <iostream>


#define IDX(i,j) (i)+4*(j)

// Matrix4x4 Declarations
/**
 * \brief Homogeneous 4-by-4 matrix
 *
 * The matrix is represented in column-major format, which enables the direct use
 * in the OpenGL Matrix Stack.
 */
struct Matrix4x4
{
    /** \brief identity matrix. **/
    Matrix4x4()
    {
      m[IDX(0,0)] = m[IDX(1,1)] = m[IDX(2,2)] = m[IDX(3,3)] = 1.f;
      m[IDX(0,1)] = m[IDX(0,2)] = m[IDX(0,3)] = m[IDX(1,0)] = m[IDX(1,2)]
          = m[IDX(1,3)] = m[IDX(2,0)] = m[IDX(2,1)] = m[IDX(2,3)] = m[IDX(3,0)]
              = m[IDX(3,1)] = m[IDX(3,2)] = 0.f;
    }
    Matrix4x4(float mat[4][4]);
    Matrix4x4(float t00, float t01, float t02, float t03, float t10, float t11,
        float t12, float t13, float t20, float t21, float t22, float t23,
        float t30, float t31, float t32, float t33);

    bool operator==(const Matrix4x4 &m2) const
    {
      /** epsilon test for equivalence **/
      const float EPS = 0.000001;
      for (int i = 0; i < 16; ++i)
        if (fabs(m[i] - m2.m[i]) > EPS) return false;
      return true;
    }

    bool operator!=(const Matrix4x4 &m2) const
    {
      /** epsilon test for equivalence **/
      const float EPS = 0.000001;
      for (int i = 0; i < 16; ++i)
        if (fabs(m[i] - m2.m[i]) > EPS) return true;
      return false;
    }
    /** \brief matrix access using row, column format **/
    float& operator()(int row, int col)
    {
      return m[IDX(row,col)];
    }
    /** \brief matrix access using row, column format **/
    float operator()(int row, int col) const
    {
      return m[IDX(row,col)];
    }
    /** \brief transpose of a matrix **/
    friend Matrix4x4 Transpose(const Matrix4x4 &);
    /** \brief outputs the matrix into a file **/
    void Print(FILE *f) const
    {
      fprintf(f, "[ ");
      for (int i = 0; i < 4; ++i)
      {
        fprintf(f, "  [ ");
        for (int j = 0; j < 4; ++j)
        {
          fprintf(f, "%f", m[IDX(i,j)]);
          if (j != 3) fprintf(f, ", ");
        }
        fprintf(f, " ]\n");
      }
      fprintf(f, " ] ");
    }
    /** \brief matrix multiplication by an operator **/
    Matrix4x4 operator*(const Matrix4x4& m2) const
    {
      return Matrix4x4::Mul(*this, m2);
    }

    /** \brief matrix multiplication m1*m2 **/
    static Matrix4x4 Mul(const Matrix4x4 &m1, const Matrix4x4 &m2)
    {
      Matrix4x4 r;
      for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
          r(i, j) = m1(i, 0) * m2(0, j) + m1(i, 1) * m2(1, j) + m1(i, 2) * m2(
              2, j) + m1(i, 3) * m2(3, j);
      return r;
    }
    /** \brief Inverse of a 4-by-4 matrix **/
    friend Matrix4x4 Inverse(const Matrix4x4 &);
    /** \brief output operator for std::cout, etc. **/
    friend std::ostream& operator<<(std::ostream& out, const Matrix4x4& mat);

    float m[16];
};

#undef IDX

// Transform Declarations
class Transform
{
  public:
    // Transform Public Methods
    Transform()
    {
      m = Matrix4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
      mInv = Matrix4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    }

    Transform(const float mat[4][4])
    {
      m = Matrix4x4(mat[0][0], mat[0][1], mat[0][2], mat[0][3], mat[1][0],
          mat[1][1], mat[1][2], mat[1][3], mat[2][0], mat[2][1], mat[2][2],
          mat[2][3], mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
      mInv = Inverse(m);
    }
    Transform(const Matrix4x4 &mat) :
      m(mat), mInv(Inverse(mat))
    {
    }
    Transform(const Matrix4x4 &mat, const Matrix4x4 &minv) :
      m(mat), mInv(minv)
    {
    }

    /** \brief initialize transform from Yaw, Pitch, Roll angles, and a translation vector.
     *
     * There are two convention for this conversion possible:
     *  0 = Yaw-Pitch-Roll
     *  1 = Yaw-Roll -Pitch
     *
     *  (Note: assuming a right-handed coordinate system:
     *      yaw    rotates around the z axis,
     *      pitch  rotates around the y axis,
     *  and roll   rotates around the x axis.)
     **/
    Transform(float yaw, float pitch, float roll, const Vector3f& t,
        int convention = 0);

    operator const float*() const;

    void Print(FILE *f) const;
    friend Transform Inverse(const Transform &t)
    {
      return Transform(t.mInv, t.m);
    }
    friend Transform Transpose(const Transform &t)
    {
      return Transform(Transpose(t.m), Transpose(t.mInv));
    }
    bool operator==(const Transform &t) const
    {
      return t.m == m && t.mInv == mInv;
    }
    bool operator!=(const Transform &t) const
    {
      return t.m != m || t.mInv != mInv;
    }
    bool operator<(const Transform &t2) const
    {
      for (uint32_t i = 0; i < 4; ++i)
        for (uint32_t j = 0; j < 4; ++j)
        {
          if (m(i, j) < t2.m(i, j)) return true;
          if (m(i, j) > t2.m(i, j)) return false;
        }
      return false;
    }
    bool IsIdentity() const
    {
      return (m(0, 0) == 1.f && m(0, 1) == 0.f && m(0, 2) == 0.f && m(0, 3)
          == 0.f && m(1, 0) == 0.f && m(1, 1) == 1.f && m(1, 2) == 0.f && m(1,
          3) == 0.f && m(2, 0) == 0.f && m(2, 1) == 0.f && m(2, 2) == 1.f && m(
          2, 3) == 0.f && m(3, 0) == 0.f && m(3, 1) == 0.f && m(3, 2) == 0.f
          && m(3, 3) == 1.f);
    }
    const Matrix4x4 &GetMatrix() const
    {
      return m;
    }
    const Matrix4x4 &GetInverseMatrix() const
    {
      return mInv;
    }
    bool HasScale() const
    {
      float la2 = (*this)(Vector3f(1, 0, 0)).LengthSquared();
      float lb2 = (*this)(Vector3f(0, 1, 0)).LengthSquared();
      float lc2 = (*this)(Vector3f(0, 0, 1)).LengthSquared();
#define NOT_ONE(x) ((x) < .999f || (x) > 1.001f)
      return (NOT_ONE(la2) || NOT_ONE(lb2) || NOT_ONE(lc2));
#undef NOT_ONE
    }
    inline Point3f operator()(const Point3f &pt) const;
    inline void operator()(const Point3f &pt, Point3f *ptrans) const;
    inline Vector3f operator()(const Vector3f &v) const;
    inline void operator()(const Vector3f &v, Vector3f *vt) const;
    inline Normal3f operator()(const Normal3f &) const;
    inline void operator()(const Normal3f &, Normal3f *nt) const;
    inline Ray operator()(const Ray &r) const;
    inline void operator()(const Ray &r, Ray *rt) const;
    /** inline RayDifferential operator()(const RayDifferential &r) const;
     inline void operator()(const RayDifferential &r, RayDifferential *rt) const;
     BBox operator()(const BBox &b) const; **/
    Transform operator*(const Transform &t2) const;
    bool SwapsHandedness() const;

    friend std::ostream& operator<<(std::ostream& out, const Transform& t)
    {
      out.width(4);
      out.precision(3);
      const Matrix4x4& m = t.GetMatrix();

      out << m;

      return out;
    }
  private:
    // Transform Private Data
    Matrix4x4 m, mInv;
};

Transform Translate(const Vector3f &delta);
Transform Scale(float x, float y, float z);
Transform RotateX(float angle);
Transform RotateY(float angle);
Transform RotateZ(float angle);
Transform Rotate(float angle, const Vector3f &axis);
Transform LookAt(const Point3f &pos, const Point3f &look, const Vector3f &up);
bool SolveLinearSystem2x2(const float A[2][2], const float B[2], float *x0,
    float *x1);
Transform Orthographic(float znear, float zfar);
Transform Perspective(float fov, float znear, float zfar);

// Transform Inline Functions
inline Point3f Transform::operator()(const Point3f &pt) const
{
  float x = pt.x, y = pt.y, z = pt.z;
  float xp = m(0, 0) * x + m(0, 1) * y + m(0, 2) * z + m(0, 3);
  float yp = m(1, 0) * x + m(1, 1) * y + m(1, 2) * z + m(1, 3);
  float zp = m(2, 0) * x + m(2, 1) * y + m(2, 2) * z + m(2, 3);
  float wp = m(3, 0) * x + m(3, 1) * y + m(3, 2) * z + m(3, 3);
  assert(wp != 0);
  if (wp == 1.)
    return Point3f(xp, yp, zp);
  else
    return Point3f(xp, yp, zp) / wp;
}

inline void Transform::operator()(const Point3f &pt, Point3f *ptrans) const
{
  float x = pt.x, y = pt.y, z = pt.z;
  ptrans->x = m(0, 0) * x + m(0, 1) * y + m(0, 2) * z + m(0, 3);
  ptrans->y = m(1, 0) * x + m(1, 1) * y + m(1, 2) * z + m(1, 3);
  ptrans->z = m(2, 0) * x + m(2, 1) * y + m(2, 2) * z + m(2, 3);
  float w = m(3, 0) * x + m(3, 1) * y + m(3, 2) * z + m(3, 3);
  if (w != 1.) *ptrans /= w;
}

inline Vector3f Transform::operator()(const Vector3f &v) const
{
  float x = v.x, y = v.y, z = v.z;
  return Vector3f(m(0, 0) * x + m(0, 1) * y + m(0, 2) * z,
      m(1, 0) * x + m(1, 1) * y + m(1, 2) * z,
      m(2, 0) * x + m(2, 1) * y + m(2, 2) * z);
}

inline void Transform::operator()(const Vector3f &v, Vector3f *vt) const
{
  float x = v.x, y = v.y, z = v.z;
  vt->x = m(0, 0) * x + m(0, 1) * y + m(0, 2) * z;
  vt->y = m(1, 0) * x + m(1, 1) * y + m(1, 2) * z;
  vt->z = m(2, 0) * x + m(2, 1) * y + m(2, 2) * z;
}

inline Normal3f Transform::operator()(const Normal3f &n) const
{
  float x = n.x, y = n.y, z = n.z;
  return Normal3f(mInv(0, 0) * x + mInv(1, 0) * y + mInv(2, 0) * z,
      mInv(0, 1) * x + mInv(1, 1) * y + mInv(2, 1) * z,
      mInv(0, 2) * x + mInv(1, 2) * y + mInv(2, 2) * z);
}

inline void Transform::operator()(const Normal3f &n, Normal3f *nt) const
{
  float x = n.x, y = n.y, z = n.z;
  nt->x = mInv(0, 0) * x + mInv(1, 0) * y + mInv(2, 0) * z;
  nt->y = mInv(0, 1) * x + mInv(1, 1) * y + mInv(2, 1) * z;
  nt->z = mInv(0, 2) * x + mInv(1, 2) * y + mInv(2, 2) * z;
}

inline Ray Transform::operator()(const Ray &r) const
{
  Ray ret = r;
  (*this)(ret.o, &ret.o);
  (*this)(ret.d, &ret.d);
  return ret;
}

inline void Transform::operator()(const Ray &r, Ray *rt) const
{
  (*this)(r.o, &rt->o);
  (*this)(r.d, &rt->d);
  if (rt != &r)
  {
    rt->mint = r.mint;
    rt->maxt = r.maxt;
    rt->time = r.time;
    rt->depth = r.depth;
  }
}

inline Transform::operator const float*() const
{
  return m.m;
}

//inline void Transform::operator()(const RayDifferential &r, RayDifferential *rt) const {
//    (*this)(Ray(r), rt);
//    rt->hasDifferentials = r.hasDifferentials;
//    (*this)(r.rxOrigin, &rt->rxOrigin);
//    (*this)(r.ryOrigin, &rt->ryOrigin);
//    (*this)(r.rxDirection, &rt->rxDirection);
//    (*this)(r.ryDirection, &rt->ryDirection);
//}


//inline RayDifferential Transform::operator()(const RayDifferential &r) const {
//    RayDifferential ret;
//    (*this)(Ray(r), &ret);
//    ret.hasDifferentials = r.hasDifferentials;
//    (*this)(r.rxOrigin, &ret.rxOrigin);
//    (*this)(r.ryOrigin, &ret.ryOrigin);
//    (*this)(r.rxDirection, &ret.rxDirection);
//    (*this)(r.ryDirection, &ret.ryDirection);
//    return ret;
//}

#endif // PBRT_CORE_TRANSFORM_H
