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

/**
 * Distinguish between points, vectors, and normals. Especially, normals need
 * other implementation of transformations.
 *
 * For further information:
 *    Matt Pharr, Greg Humphreys.
 *    Physically Based Rendering - From Theory to Implementation.
 *    Morgan Kaufmann, 2010.
 */

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <stdint.h>
#include <cassert>
#include <cmath>
#include <iostream>

// Global Inline Functions
inline float Lerp(float t, float v1, float v2) {
  return (1.f - t) * v1 + t * v2;
}

inline float Clamp(float val, float low, float high) {
  if (val < low)
    return low;
  else if (val > high)
    return high;
  else
    return val;
}

inline int Clamp(int val, int low, int high) {
  if (val < low)
    return low;
  else if (val > high)
    return high;
  else
    return val;
}

inline int Mod(int a, int b) {
  int n = int(a / b);
  a -= n * b;
  if (a < 0) a += b;
  return a;
}

inline float Radians(float deg) {
  return ((float)M_PI / 180.f) * deg;
}

inline float Degrees(float rad) {
  return (180.f / (float)M_PI) * rad;
}

inline float Log2(float x) {
  static float invLog2 = 1.f / logf(2.f);
  return logf(x) * invLog2;
}

inline int Floor2Int(float val);
inline int Log2Int(float v) {
  return Floor2Int(Log2(v));
}

inline bool IsPowerOf2(int v) {
  return (v & (v - 1)) == 0;
}

inline uint32_t RoundUpPow2(uint32_t v) {
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  return v + 1;
}

inline int Floor2Int(float val) {
  return (int)floorf(val);
}

inline int Round2Int(float val) {
  return Floor2Int(val + 0.5f);
}

inline int Float2Int(float val) {
  return (int)val;
}

inline int Ceil2Int(float val) {
  return (int)ceilf(val);
}

struct Vector3f;
struct Point3f;
struct Normal3f;

// Geometry Declarations
struct Vector3f {
 public:
  // Vector3f Public Methods
  Vector3f() { x = y = z = 0.f; }
  Vector3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) { assert(!HasNaNs()); }
  bool HasNaNs() const { return std::isnan(x) || std::isnan(y) || std::isnan(z); }
  explicit Vector3f(const Point3f &p);
#ifndef NDEBUG
  // The default versions of these are fine for release builds; for debug
  // we define them so that we can add the assert checks.
  Vector3f(const Vector3f &v) {
    assert(!v.HasNaNs());
    x = v.x;
    y = v.y;
    z = v.z;
  }

  Vector3f &operator=(const Vector3f &v) {
    assert(!v.HasNaNs());
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
  }
#endif  // !NDEBUG
  Vector3f operator+(const Vector3f &v) const {
    assert(!v.HasNaNs());
    return Vector3f(x + v.x, y + v.y, z + v.z);
  }

  Vector3f &operator+=(const Vector3f &v) {
    assert(!v.HasNaNs());
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }
  Vector3f operator-(const Vector3f &v) const {
    assert(!v.HasNaNs());
    return Vector3f(x - v.x, y - v.y, z - v.z);
  }

  Vector3f &operator-=(const Vector3f &v) {
    assert(!v.HasNaNs());
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }
  Vector3f operator*(float f) const { return Vector3f(f * x, f * y, f * z); }

  Vector3f &operator*=(float f) {
    assert(!std::isnan(f));
    x *= f;
    y *= f;
    z *= f;
    return *this;
  }
  Vector3f operator/(float f) const {
    assert(f != 0);
    float inv = 1.f / f;
    return Vector3f(x * inv, y * inv, z * inv);
  }

  Vector3f &operator/=(float f) {
    assert(f != 0);
    float inv = 1.f / f;
    x *= inv;
    y *= inv;
    z *= inv;
    return *this;
  }
  Vector3f operator-() const { return Vector3f(-x, -y, -z); }
  float operator[](int i) const {
    assert(i >= 0 && i <= 2);
    return (&x)[i];
  }

  float &operator[](int i) {
    assert(i >= 0 && i <= 2);
    return (&x)[i];
  }
  float LengthSquared() const { return x * x + y * y + z * z; }
  float Length() const { return sqrtf(LengthSquared()); }
  explicit Vector3f(const Normal3f &n);

  bool operator==(const Vector3f &v) const { return x == v.x && y == v.y && z == v.z; }
  bool operator!=(const Vector3f &v) const { return x != v.x || y != v.y || z != v.z; }

  friend std::ostream &operator<<(std::ostream &out, const Vector3f &v) {
    out.width(4);
    out.precision(3);
    out << v.x << ", " << v.y << ", " << v.z;
    return out;
  }

  // Vector3f Public Data
  float x, y, z;
};

struct Point3f {
 public:
  // Point3f Public Methods
  Point3f() { x = y = z = 0.f; }
  Point3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {
    if (HasNaNs()) std::cerr << *this << std::endl;
    assert(!HasNaNs());
  }
  explicit Point3f(const Vector3f &v) : x(v.x), y(v.y), z(v.z) {}

#ifndef NDEBUG
  Point3f(const Point3f &p) {
    if (p.HasNaNs()) std::cerr << p << std::endl;
    assert(!p.HasNaNs());
    x = p.x;
    y = p.y;
    z = p.z;
  }

  Point3f &operator=(const Point3f &p) {
    if (p.HasNaNs()) std::cerr << p << std::endl;
    assert(!p.HasNaNs());
    x = p.x;
    y = p.y;
    z = p.z;
    return *this;
  }
#endif  // !NDEBUG
  Point3f operator+(const Vector3f &v) const {
    assert(!v.HasNaNs());
    return Point3f(x + v.x, y + v.y, z + v.z);
  }

  Point3f &operator+=(const Vector3f &v) {
    assert(!v.HasNaNs());
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }
  Vector3f operator-(const Point3f &p) const {
    assert(!p.HasNaNs());
    return Vector3f(x - p.x, y - p.y, z - p.z);
  }

  Point3f operator-(const Vector3f &v) const {
    assert(!v.HasNaNs());
    return Point3f(x - v.x, y - v.y, z - v.z);
  }

  Point3f &operator-=(const Vector3f &v) {
    assert(!v.HasNaNs());
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }
  Point3f &operator+=(const Point3f &p) {
    assert(!p.HasNaNs());
    x += p.x;
    y += p.y;
    z += p.z;
    return *this;
  }
  Point3f operator+(const Point3f &p) const {
    assert(!p.HasNaNs());
    return Point3f(x + p.x, y + p.y, z + p.z);
  }
  Point3f operator*(float f) const { return Point3f(f * x, f * y, f * z); }
  Point3f &operator*=(float f) {
    x *= f;
    y *= f;
    z *= f;
    return *this;
  }
  Point3f operator/(float f) const {
    float inv = 1.f / f;
    return Point3f(inv * x, inv * y, inv * z);
  }
  Point3f &operator/=(float f) {
    float inv = 1.f / f;
    x *= inv;
    y *= inv;
    z *= inv;
    return *this;
  }
  float operator[](int i) const {
    //      assert(i >= 0 && i <= 2); ** CUDA doesn't allow assertion in kernels **
    return (&x)[i];
  }

  float &operator[](int i) {
    //      assert(i >= 0 && i <= 2); ** CUDA doesn't allow assertion in kernels **
    return (&x)[i];
  }
  bool HasNaNs() const { return std::isnan(x) || std::isnan(y) || std::isnan(z); }

  bool operator==(const Point3f &p) const { /** todo: epsilon tests? **/
    return x == p.x && y == p.y && z == p.z;
  }

  bool operator!=(const Point3f &p) const { /** todo: epsilon tests? **/
    return x != p.x || y != p.y || z != p.z;
  }

  friend std::ostream &operator<<(std::ostream &out, const Point3f &p) {
    out.width(4);
    out.precision(3);
    out << p.x << ", " << p.y << ", " << p.z;
    return out;
  }
  // Point3f Public Data
  float x, y, z;
};

struct Normal3f {
 public:
  // Normal3f Public Methods
  Normal3f() { x = y = z = 0.f; }
  Normal3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) { assert(!HasNaNs()); }
  Normal3f operator-() const { return Normal3f(-x, -y, -z); }
  Normal3f operator+(const Normal3f &n) const {
    assert(!n.HasNaNs());
    return Normal3f(x + n.x, y + n.y, z + n.z);
  }

  Normal3f &operator+=(const Normal3f &n) {
    assert(!n.HasNaNs());
    x += n.x;
    y += n.y;
    z += n.z;
    return *this;
  }
  Normal3f operator-(const Normal3f &n) const {
    assert(!n.HasNaNs());
    return Normal3f(x - n.x, y - n.y, z - n.z);
  }

  Normal3f &operator-=(const Normal3f &n) {
    assert(!n.HasNaNs());
    x -= n.x;
    y -= n.y;
    z -= n.z;
    return *this;
  }
  bool HasNaNs() const { return std::isnan(x) || std::isnan(y) || std::isnan(z); }
  Normal3f operator*(float f) const { return Normal3f(f * x, f * y, f * z); }

  Normal3f &operator*=(float f) {
    x *= f;
    y *= f;
    z *= f;
    return *this;
  }
  Normal3f operator/(float f) const {
    assert(f != 0);
    float inv = 1.f / f;
    return Normal3f(x * inv, y * inv, z * inv);
  }

  Normal3f &operator/=(float f) {
    assert(f != 0);
    float inv = 1.f / f;
    x *= inv;
    y *= inv;
    z *= inv;
    return *this;
  }
  float LengthSquared() const { return x * x + y * y + z * z; }
  float Length() const { return sqrtf(LengthSquared()); }

#ifndef NDEBUG
  Normal3f(const Normal3f &n) {
    assert(!n.HasNaNs());
    x = n.x;
    y = n.y;
    z = n.z;
  }

  Normal3f &operator=(const Normal3f &n) {
    assert(!n.HasNaNs());
    x = n.x;
    y = n.y;
    z = n.z;
    return *this;
  }
#endif  // !NDEBUG
  explicit Normal3f(const Vector3f &v) : x(v.x), y(v.y), z(v.z) { assert(!v.HasNaNs()); }
  float operator[](int i) const {
    assert(i >= 0 && i <= 2);
    return (&x)[i];
  }

  float &operator[](int i) {
    assert(i >= 0 && i <= 2);
    return (&x)[i];
  }

  bool operator==(const Normal3f &n) const { return x == n.x && y == n.y && z == n.z; }
  bool operator!=(const Normal3f &n) const { return x != n.x || y != n.y || z != n.z; }

  // Normal3f Public Data
  float x, y, z;
};

struct Ray {
 public:
  // Ray Public Methods
  Ray() : mint(0.f), maxt(INFINITY), time(0.f), depth(0) {}
  Ray(const Point3f &origin, const Vector3f &direction, float start, float end = INFINITY, float t = 0.f, int d = 0)
      : o(origin), d(direction), mint(start), maxt(end), time(t), depth(d) {}
  Ray(const Point3f &origin, const Vector3f &direction, const Ray &parent, float start, float end = INFINITY)
      : o(origin), d(direction), mint(start), maxt(end), time(parent.time), depth(parent.depth + 1) {}
  Point3f operator()(float t) const { return o + d * t; }
  bool HasNaNs() const { return (o.HasNaNs() || d.HasNaNs() || std::isnan(mint) || std::isnan(maxt)); }

  // Ray Public Data
  Point3f o;
  Vector3f d;
  mutable float mint, maxt;
  float time;
  int depth;
};

// Geometry Inline Functions
inline Vector3f::Vector3f(const Point3f &p) : x(p.x), y(p.y), z(p.z) {
  assert(!HasNaNs());
}

inline Vector3f operator*(float f, const Vector3f &v) {
  return v * f;
}
inline float Dot(const Vector3f &v1, const Vector3f &v2) {
  assert(!v1.HasNaNs() && !v2.HasNaNs());
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

inline float AbsDot(const Vector3f &v1, const Vector3f &v2) {
  assert(!v1.HasNaNs() && !v2.HasNaNs());
  return fabsf(Dot(v1, v2));
}

inline Vector3f Cross(const Vector3f &v1, const Vector3f &v2) {
  assert(!v1.HasNaNs() && !v2.HasNaNs());
  double v1x = v1.x, v1y = v1.y, v1z = v1.z;
  double v2x = v2.x, v2y = v2.y, v2z = v2.z;
  return Vector3f((v1y * v2z) - (v1z * v2y), (v1z * v2x) - (v1x * v2z), (v1x * v2y) - (v1y * v2x));
}

inline Vector3f Cross(const Vector3f &v1, const Normal3f &v2) {
  assert(!v1.HasNaNs() && !v2.HasNaNs());
  double v1x = v1.x, v1y = v1.y, v1z = v1.z;
  double v2x = v2.x, v2y = v2.y, v2z = v2.z;
  return Vector3f((v1y * v2z) - (v1z * v2y), (v1z * v2x) - (v1x * v2z), (v1x * v2y) - (v1y * v2x));
}

inline Vector3f Cross(const Normal3f &v1, const Vector3f &v2) {
  assert(!v1.HasNaNs() && !v2.HasNaNs());
  double v1x = v1.x, v1y = v1.y, v1z = v1.z;
  double v2x = v2.x, v2y = v2.y, v2z = v2.z;
  return Vector3f((v1y * v2z) - (v1z * v2y), (v1z * v2x) - (v1x * v2z), (v1x * v2y) - (v1y * v2x));
}

inline Vector3f Normalize(const Vector3f &v) {
  return v / v.Length();
}

// inline void CoordinateSystem(const Vector3f &v1, Vector3f *v2, Vector3f *v3)
//{
//  if (fabsf(v1.x) > fabsf(v1.y))
//  {
//    float invLen = 1.f / sqrtf(v1.x * v1.x + v1.z * v1.z);
//    *v2 = Vector3f(-v1.z * invLen, 0.f, v1.x * invLen);
//  }
//  else
//  {
//    float invLen = 1.f / sqrtf(v1.y * v1.y + v1.z * v1.z);
//    *v2 = Vector3f(0.f, v1.z * invLen, -v1.y * invLen);
//  }
//  *v3 = Cross(v1, *v2);
//}

inline float Distance(const Point3f &p1, const Point3f &p2) {
  return (p1 - p2).Length();
}

inline float DistanceSquared(const Point3f &p1, const Point3f &p2) {
  return (p1 - p2).LengthSquared();
}

inline Point3f operator*(float f, const Point3f &p) {
  assert(!p.HasNaNs());
  return p * f;
}

inline Normal3f operator*(float f, const Normal3f &n) {
  return Normal3f(f * n.x, f * n.y, f * n.z);
}

inline Normal3f Normalize(const Normal3f &n) {
  return n / n.Length();
}

inline Vector3f::Vector3f(const Normal3f &n) : x(n.x), y(n.y), z(n.z) {
  assert(!n.HasNaNs());
}

inline float Dot(const Normal3f &n1, const Vector3f &v2) {
  assert(!n1.HasNaNs() && !v2.HasNaNs());
  return n1.x * v2.x + n1.y * v2.y + n1.z * v2.z;
}

inline float Dot(const Vector3f &v1, const Normal3f &n2) {
  assert(!v1.HasNaNs() && !n2.HasNaNs());
  return v1.x * n2.x + v1.y * n2.y + v1.z * n2.z;
}

inline float Dot(const Normal3f &n1, const Normal3f &n2) {
  assert(!n1.HasNaNs() && !n2.HasNaNs());
  return n1.x * n2.x + n1.y * n2.y + n1.z * n2.z;
}

inline float AbsDot(const Normal3f &n1, const Vector3f &v2) {
  assert(!n1.HasNaNs() && !v2.HasNaNs());
  return fabsf(n1.x * v2.x + n1.y * v2.y + n1.z * v2.z);
}

inline float AbsDot(const Vector3f &v1, const Normal3f &n2) {
  assert(!v1.HasNaNs() && !n2.HasNaNs());
  return fabsf(v1.x * n2.x + v1.y * n2.y + v1.z * n2.z);
}

inline float AbsDot(const Normal3f &n1, const Normal3f &n2) {
  assert(!n1.HasNaNs() && !n2.HasNaNs());
  return fabsf(n1.x * n2.x + n1.y * n2.y + n1.z * n2.z);
}

inline Normal3f Faceforward(const Normal3f &n, const Vector3f &v) {
  return (Dot(n, v) < 0.f) ? -n : n;
}

inline Normal3f Faceforward(const Normal3f &n, const Normal3f &n2) {
  return (Dot(n, n2) < 0.f) ? -n : n;
}

inline Vector3f Faceforward(const Vector3f &v, const Vector3f &v2) {
  return (Dot(v, v2) < 0.f) ? -v : v;
}

inline Vector3f Faceforward(const Vector3f &v, const Normal3f &n2) {
  return (Dot(v, n2) < 0.f) ? -v : v;
}

inline Vector3f SphericalDirection(float sintheta, float costheta, float phi) {
  return Vector3f(sintheta * cosf(phi), sintheta * sinf(phi), costheta);
}

inline Vector3f SphericalDirection(float sintheta, float costheta, float phi, const Vector3f &x, const Vector3f &y,
                                   const Vector3f &z) {
  return sintheta * cosf(phi) * x + sintheta * sinf(phi) * y + costheta * z;
}

inline float SphericalTheta(const Vector3f &v) {
  return acosf(Clamp(v.z, -1.f, 1.f));
}

inline float SphericalPhi(const Vector3f &v) {
  float p = atan2f(v.y, v.x);
  return (p < 0.f) ? p + 2.f * M_PI : p;
}
#endif  // GEOMETRY_H
