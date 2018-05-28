/*****************************************************************************\
 rose

 ViewFrustrum.cpp
 Author: behley

 \*****************************************************************************/

#include "ViewFrustum.h"
#include <GL/gl.h>
#include <cassert>

ViewFrustum::ViewFrustum() :
	frustrum(0)
{
	frustrum = new double*[6];
	for (unsigned int i = 0; i < 6; ++i)
		frustrum[i] = new double[4];
}

ViewFrustum::~ViewFrustum()
{
	for (unsigned int i = 0; i < 6; ++i)
		delete[] frustrum[i];
	delete[] frustrum;
}

bool ViewFrustum::isInside(const Point3f& point) const
{
	return isInside(point.x, point.y, point.z);
}

bool ViewFrustum::isInside(double x, double y, double z) const
{
//	if ((frustrum[Back][A] * x + frustrum[Back][B] * y + frustrum[Back][C] * z + frustrum[Back][D])
//			< 0.)
//	{
//		return false;
//	}

	if ((frustrum[Front][A] * x + frustrum[Front][B] * y + frustrum[Front][C] * z + frustrum[Front][D])
			< 0.)
	{
		return false;
	}

	return true;
}

void ViewFrustum::update()
{
	double ProjM[16];
	double ModM[16];
	double clip[16];

	glGetDoublev(GL_PROJECTION_MATRIX, ProjM);
	glGetDoublev(GL_MODELVIEW_MATRIX, ModM);

	clip[0] = ModM[0] * ProjM[0] + ModM[1] * ProjM[4] + ModM[2] * ProjM[8] + ModM[3] * ProjM[12];
	clip[1] = ModM[0] * ProjM[1] + ModM[1] * ProjM[5] + ModM[2] * ProjM[9] + ModM[3] * ProjM[13];
	clip[2] = ModM[0] * ProjM[2] + ModM[1] * ProjM[6] + ModM[2] * ProjM[10] + ModM[3] * ProjM[14];
	clip[3] = ModM[0] * ProjM[3] + ModM[1] * ProjM[7] + ModM[2] * ProjM[11] + ModM[3] * ProjM[15];
	clip[4] = ModM[4] * ProjM[0] + ModM[5] * ProjM[4] + ModM[6] * ProjM[8] + ModM[7] * ProjM[12];
	clip[5] = ModM[4] * ProjM[1] + ModM[5] * ProjM[5] + ModM[6] * ProjM[9] + ModM[7] * ProjM[13];
	clip[6] = ModM[4] * ProjM[2] + ModM[5] * ProjM[6] + ModM[6] * ProjM[10] + ModM[7] * ProjM[14];
	clip[7] = ModM[4] * ProjM[3] + ModM[5] * ProjM[7] + ModM[6] * ProjM[11] + ModM[7] * ProjM[15];
	clip[8] = ModM[8] * ProjM[0] + ModM[9] * ProjM[4] + ModM[10] * ProjM[8] + ModM[11] * ProjM[12];
	clip[9] = ModM[8] * ProjM[1] + ModM[9] * ProjM[5] + ModM[10] * ProjM[9] + ModM[11] * ProjM[13];
	clip[10] = ModM[8] * ProjM[2] + ModM[9] * ProjM[6] + ModM[10] * ProjM[10] + ModM[11] * ProjM[14];
	clip[11] = ModM[8] * ProjM[3] + ModM[9] * ProjM[7] + ModM[10] * ProjM[11] + ModM[11] * ProjM[15];
	clip[12] = ModM[12] * ProjM[0] + ModM[13] * ProjM[4] + ModM[14] * ProjM[8] + ModM[15] * ProjM[12];
	clip[13] = ModM[12] * ProjM[1] + ModM[13] * ProjM[5] + ModM[14] * ProjM[9] + ModM[15] * ProjM[13];
	clip[14] = ModM[12] * ProjM[2] + ModM[13] * ProjM[6] + ModM[14] * ProjM[10] + ModM[15]
			* ProjM[14];
	clip[15] = ModM[12] * ProjM[3] + ModM[13] * ProjM[7] + ModM[14] * ProjM[11] + ModM[15]
			* ProjM[15];

	frustrum[Right][A] = clip[3] - clip[0];
	frustrum[Right][B] = clip[7] - clip[4];
	frustrum[Right][C] = clip[11] - clip[8];
	frustrum[Right][D] = clip[15] - clip[12];
	normalizePlane(Right);

	frustrum[Left][A] = clip[3] + clip[0];
	frustrum[Left][B] = clip[7] + clip[4];
	frustrum[Left][C] = clip[11] + clip[8];
	frustrum[Left][D] = clip[15] + clip[12];
	normalizePlane(Left);

	frustrum[Bottom][A] = clip[3] + clip[1];
	frustrum[Bottom][B] = clip[7] + clip[5];
	frustrum[Bottom][C] = clip[11] + clip[9];
	frustrum[Bottom][D] = clip[15] + clip[13];
	normalizePlane(Bottom);

	frustrum[Top][A] = clip[3] - clip[1];
	frustrum[Top][B] = clip[7] - clip[5];
	frustrum[Top][C] = clip[11] - clip[9];
	frustrum[Top][D] = clip[15] - clip[13];
	normalizePlane(Top);

	frustrum[Back][A] = clip[3] - clip[2];
	frustrum[Back][B] = clip[7] - clip[6];
	frustrum[Back][C] = clip[11] - clip[10];
	frustrum[Back][D] = clip[15] - clip[14];
	normalizePlane(Back);

	frustrum[Front][A] = clip[3] + clip[2];
	frustrum[Front][B] = clip[7] + clip[6];
	frustrum[Front][C] = clip[11] + clip[10];
	frustrum[Front][D] = clip[15] + clip[14];
	normalizePlane(Front);
}

void ViewFrustum::normalizePlane(int pPlane)
{
	double Magnitude = sqrtf(Math::sqr(frustrum[pPlane][A]) + Math::sqr(frustrum[pPlane][B])
			+ Math::sqr(frustrum[pPlane][C]));

	frustrum[pPlane][A] = frustrum[pPlane][A] / Magnitude;
	frustrum[pPlane][B] = frustrum[pPlane][B] / Magnitude;
	frustrum[pPlane][C] = frustrum[pPlane][C] / Magnitude;
	frustrum[pPlane][D] = frustrum[pPlane][D] / Magnitude;
}
