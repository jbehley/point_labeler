/** \brief some common transformations **/
#ifndef RVCOORDINATESYSTEM_H_
#define RVCOORDINATESYSTEM_H_

#include "transform.h"

namespace rv
{
/** \brief coordinate transform from RoSe to OpenGL coordinates. **/
Transform RoSe2GL = Transform(
    Matrix4x4( 0, -1, 0, 0,
               0,  0, 1, 0,
              -1,  0, 0, 0,
               0,  0, 0, 1));

/** \brief coordinate transform from RoSe to OpenGL coordinates. **/
Transform GL2RoSe = Transform(
    Matrix4x4( 0, 0,-1, 0,
              -1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 0, 1));
}

#endif /* COORDINATESYSTEM_H_ */
