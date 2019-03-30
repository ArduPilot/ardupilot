#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

/*
  macros to allow code to build on multiple platforms more easily
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
/*
  allow double maths on Linux and SITL to avoid problems with system headers
 */
#define ALLOW_DOUBLE_MATH_FUNCTIONS
#endif

#if !defined(ALLOW_DOUBLE_MATH_FUNCTIONS)
/* give warnings if we use double precision maths functions without
   specifying ALLOW_DOUBLE_TRIG_FUNCTIONS. Code should use the
   equivalent f function instead (eg. use cosf() instead of
   cos()). Individual cpp files that really do need double precision
   should define ALLOW_DOUBLE_TRIG_FUNCTIONS before including
   AP_Math.h
*/
#define sin(x) DO_NOT_USE_DOUBLE_MATHS()
#define cos(x) DO_NOT_USE_DOUBLE_MATHS()
#define tan(x) DO_NOT_USE_DOUBLE_MATHS()
#define acos(x) DO_NOT_USE_DOUBLE_MATHS()
#define asin(x) DO_NOT_USE_DOUBLE_MATHS()
#define atan(x) DO_NOT_USE_DOUBLE_MATHS()
#define atan2(x,y) DO_NOT_USE_DOUBLE_MATHS()
#define exp(x) DO_NOT_USE_DOUBLE_MATHS()
#define pow(x,y) DO_NOT_USE_DOUBLE_MATHS()
#define sqrt(x) DO_NOT_USE_DOUBLE_MATHS()
#define log2(x) DO_NOT_USE_DOUBLE_MATHS()
#define log10(x) DO_NOT_USE_DOUBLE_MATHS()
#define ceil(x) DO_NOT_USE_DOUBLE_MATHS()
#define floor(x) DO_NOT_USE_DOUBLE_MATHS()
#define round(x) DO_NOT_USE_DOUBLE_MATHS()
#define fmax(x,y) DO_NOT_USE_DOUBLE_MATHS()
#if !HAL_WITH_UAVCAN
// we should do log() and fabs() as well, but can't because of a conflict in uavcan
#define log(x) DO_NOT_USE_DOUBLE_MATHS()
#define fabs(x) DO_NOT_USE_DOUBLE_MATHS()
#endif
#endif

