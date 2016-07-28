#pragma once

#include <cmath>

#include <AP_HAL/AP_HAL.h>

// Double precision math must be activated
#if defined(DBL_MATH) && CONFIG_HAL_BOARD == HAL_BOARD_LINUX
  #ifndef M_PI
    #define M_PI      (3.14159265358979323846)
  #endif

  #ifndef M_PI_2
    #define M_PI_2    (M_PI / 2)
  #endif

  #define M_GOLDEN    1.618033988749894
#else    // Standard single precision math
  #ifdef M_PI
    #undef M_PI
  #endif
  #define M_PI      (3.141592653589793f)

  #ifdef M_PI_2
    #undef M_PI_2
  #endif
  #define M_PI_2    (M_PI / 2)

  #define M_GOLDEN  1.6180339f
#endif

#define M_2PI         (M_PI * 2)

// MATH_CHECK_INDEXES modifies some objects (e.g. SoloGimbalEKF) to
// include more debug information.  It is also used by some functions
// to add extra code for debugging purposes. If you wish to activate
// this, do it here or as part of the top-level Makefile -
// e.g. Tools/Replay/Makefile
#ifndef MATH_CHECK_INDEXES
  #define MATH_CHECK_INDEXES 0
#endif

#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

// GPS Specific double precision conversions
// The precision here does matter when using the wsg* functions for converting
// between LLH and ECEF coordinates.
#define DEG_TO_RAD_DOUBLE 0.0174532925199432954743716805978692718781530857086181640625
#define RAD_TO_DEG_DOUBLE 57.29577951308232286464772187173366546630859375

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
#define USEC_PER_MSEC   1000ULL
