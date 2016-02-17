#pragma once

#include <math.h>

/*
 * Redefine GCC's standard macro constants as float instead of double
* For PX4 and co this is useful because they have no native double precision unit.
*/
#if CONFIG_HAL_BOARD != HAL_BOARD_LINUX
  #ifdef M_PI
    #undef  M_PI
  #endif
  #ifdef M_PI_2
    #undef  M_PI_2
  #endif
 
  #define M_PI          (3.141592653589793f)
  #define M_PI_2        (M_PI / 2)
#endif
 
#define M_2PI           (M_PI * 2)
 
#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)
 
#define RadiansToCentiDegrees(x) (static_cast<float>(x) * RAD_TO_DEG * static_cast<float>(100))
 
// acceleration due to gravity in m/s/s
#define GRAVITY_MSS     9.80665f
 
// radius of earth in meters
#define RADIUS_OF_EARTH 6378100
 
// convert a longitude or latitude point to meters or centimeteres.
// Note: this does not include the longitude scaling which is dependent upon location
#define LATLON_TO_M     0.01113195f
#define LATLON_TO_CM    1.113195f
 
// Semi-major axis of the Earth, in meters.
#define WGS84_A         6378137.0
//Inverse flattening of the Earth
#define WGS84_IF        298.257223563
// The flattening of the Earth
#define WGS84_F         (1.0 / WGS84_IF)
// Semi-minor axis of the Earth in meters
#define WGS84_B         (WGS84_A * (1 - WGS84_F))
// Eccentricity of the Earth
#define WGS84_E         (sqrt(2 * WGS84_F - WGS84_F * WGS84_F))
 
#define NSEC_PER_SEC    1000000000ULL
#define NSEC_PER_USEC   1000ULL
#define USEC_PER_SEC    1000000ULL