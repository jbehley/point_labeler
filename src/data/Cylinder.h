/** \brief representation of a cylinder. **/

#ifndef CYLINDER_H_
#define CYLINDER_H_

#include "geometry.h"

class Cylinder
{
  public:
    Cylinder()
    : startPointInitialized(false), endPointInitialized(false), radius(0.25f), label(0)
    {

    }

   Point3f s, e;
   bool startPointInitialized, endPointInitialized;
   float radius;
   uint32_t label;
};

#endif /* CYLINDER_H_ */
