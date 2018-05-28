/*****************************************************************************\
 \headerfile Math.h

 \brief some math utility functions

 \author behley

 \*****************************************************************************/

#ifndef MATH_H_
#define MATH_H_

#include <cmath>

class Math
{
	public:
		static double PI;
		/** \brief convert an angle given in degrees to radian **/
		inline static double deg2rad(double deg) { 	return deg * PI / 180.0;	}
		/** \brief convert an angle given in radian to degrees **/
		inline static double rad2deg(double rad) 	{ return rad * 180.0 / PI; 	}

		/** \brief rounding to the nearest integer. **/
		inline static double round(double a) { return std::floor(a+0.5); }

		inline static double sqr(double a) { return a*a; }

	protected:
		Math()
		{
		}
};

#endif /* MATH_H_ */
