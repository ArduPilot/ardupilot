#pragma once

#include <AP_HAL/AP_HAL.h> // CONFIG_HAL_BOARD
#include <math.h>          // Macro definitions


#ifdef M_PI
  #undef M_PI
#endif

#ifdef M_PI_2
  #undef M_PI_2
#endif

#define M_PI          (3.141592653589793f)
#define M_PI_2        (M_PI / 2)

#define M_2PI         (M_PI * 2)
