#include "ColorGL.h"
#include <cmath>
#include "Math.h"

const ColorGL ColorGL::WHITE = ColorGL(1.0f, 1.0f, 1.0f);
const ColorGL ColorGL::BLACK = ColorGL(0.0f, 0.0f, 0.0f);
const ColorGL ColorGL::RED = ColorGL(1.0f, 0.0f, 0.0f);
const ColorGL ColorGL::GREEN = ColorGL(0.0f, 1.0f, 0.0f);
const ColorGL ColorGL::BLUE = ColorGL(0.0f, 0.0f, 1.0f);
const ColorGL ColorGL::YELLOW = ColorGL(1.0f, 1.0f, 0.0f);
const ColorGL ColorGL::PINK = ColorGL(1.0f, 0.0f, 1.0f);
const ColorGL ColorGL::ORANGE = ColorGL(1.0f, 0.65f, 0.0f);
const ColorGL ColorGL::CYAN = ColorGL(0.0f, 1.0f, 1.0f);
const ColorGL ColorGL::GOLD = ColorGL(0.85f, 0.65, 0.13f);

ColorGL ColorGL::FromRGB(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
  return ColorGL((float) r / 255.f, (float) g / 255.f, (float) b / 255.f,
      (float) a / 255.f);
}

ColorGL ColorGL::FromHSV(float h, float s, float v, float a)
{
  if (std::abs(s) < 0.00001) return ColorGL(v, v, v, a);

  float sixty = 1.04719755f; // == 60 deg
  int h_i = std::floor(h / sixty); /** integer part of h **/
  float f = h / sixty - h_i; /** fractional part of h **/
  float p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f));

  switch (h_i)
  {
    case 0:
    case 6:
      return ColorGL(v, t, p);
    case 1:
      return ColorGL(q, v, p);
    case 2:
      return ColorGL(p, v, t);
    case 3:
      return ColorGL(p, q, v);
    case 4:
      return ColorGL(t, p, v);
    case 5:
      return ColorGL(v, p, q);
    default:
      return ColorGL(0.f, 0.f, 0.f);
  }
}

//RandomColorGenerator::RandomColorGenerator(uint32_t seed)
//: rng(seed)
//{
//
//}
//
///** \brief generates a random color **/
//ColorGL RandomColorGenerator::nextColor()
//{
//  double rnd = rng.getFloat();
//  ColorGL color = ColorGL::FromHSV(2.0f*Math::PI*rnd, 1.0f, 1.0f);
//
//  return color;
//}
