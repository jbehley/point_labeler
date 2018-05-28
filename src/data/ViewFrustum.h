/*****************************************************************************\
 \brief class representing a ViewFrustum

 Using the method update(), the current ViewFrustum can be calculated using the
 current Projection and ModelView Matrix from OpenGL. With isInside() can be
 tested, whether a point lies inside the ViewFrustum.

 The ViewFrustrum is used to cull points that are definitely not visible.

	(The code for calculating the view frustrum is adopted from the corresponding
	DGL Wiki tutorial article. See: http://wiki.delphigl.com/index.php/Tutorial_Frustum_Culling)

 \header ViewFrustrum.h

 \author behley

 \*****************************************************************************/

#ifndef VIEWFRUSTUM_H_
#define VIEWFRUSTUM_H_

#include "Math.h"
#include <cmath>
#include "geometry.h"

class ViewFrustum
{
	public:
		ViewFrustum();
		~ViewFrustum();
		/** \brief Updates the view frustrum using the Projection and ModelView matrix of OpenGL. **/
		void update();

		bool isInside(const Point3f& point) const;
		bool isInside(double x, double y, double z) const;


	protected:

		void normalizePlane(int pPlane);


		double** frustrum;

		/** some constants **/
		static const int Right = 1;
		static const int Left = 0;
		static const int Bottom = 3;
		static const int Top = 2;
		static const int Back = 4;
		static const int Front = 5;
		static const int A = 0;
		static const int B = 1;
		static const int C = 2;
		static const int D = 3;
};

#endif /* VIEWFRUSTUM_H_ */
