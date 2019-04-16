
#include <iostream>
#include <stdint.h>

int32_t main(int32_t argc, char** argv)
{

  if(argc < 2)
  {
    std::cout << "Usage: ./init_instances <folder> [options]" << std::endl;
    return 1;
  }

  // idea: use sam algorithm for static and moving parts.

  // quad tree per scan / and previous scan.
  // clustering for each scan. Associate with ids from prior scan via "bounding box" overlap or nearest neighbor association.




  return 0;
}

