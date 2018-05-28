/**
 * \brief ColorGL represents a RGB color given by [0.0,1.0] values and an alpha value also in [0.0,1.0].
 *
 * This class represents RGB colors and provides some helper functions.
 *
 * Usage example setting color to red:
 *
 * glColor3fv(ColorGL::RED);
 *
 *
 * \author behley
 */

#ifndef COLORGL_H_
#define COLORGL_H_

#include <cassert>
#include <stdint.h>

class ColorGL
{
  public:
    ColorGL() :
      R(0.0f), G(0.0f), B(0.0f), A(1.0f)
    {

    }

    ColorGL(float r, float g, float b, float a = 1.0f) :
      R(r), G(g), B(b), A(a)
    {

    }

    /** \brief convert to float*
     *  NOTE: array is valid as long as object is not destroyed.
     */
    operator float*()
    {
      return &R;
    }

    operator const float*() const
    {
      return &R;
    }

    /** \brief access to the RGB values as in an array **/
    inline float operator[](int i)
    {
      return (&R)[i];
    }

    /** \brief generate Color from HSV values.
     *	The values must be provided as follows:
     *		- hue in [0,2pi],
     *		- saturation in [0.0, 1.0f]
     *		- value in [0.0, 1.0f]
     **/
    static ColorGL FromHSV(float h, float s, float v, float a = 1.0f);
    /** \brief generate Color from RGB values in [0, 255] **/
    static ColorGL FromRGB(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);

    float R, G, B, A;

    /** some common predefined colors. **/
    static const ColorGL WHITE;
    static const ColorGL BLACK;
    static const ColorGL RED;
    static const ColorGL GREEN;
    static const ColorGL BLUE;
    static const ColorGL YELLOW;
    static const ColorGL PINK;
    static const ColorGL ORANGE;
    static const ColorGL CYAN;
    static const ColorGL GOLD;
};

//class RandomColorGenerator
//{
//  public:
//    RandomColorGenerator(uint32_t seed = 0);
//
//    /** \brief generates a random color **/
//    ColorGL nextColor();
//
//  protected:
//    RoSe::Random rng;
//};
#endif /* COLORGL_H */
