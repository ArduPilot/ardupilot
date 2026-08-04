#pragma once

// board-specific values generated into a hwdef fragment
#if __has_include(<hwdef_heater.h>)
#include <hwdef_heater.h>
#endif

#ifndef HAL_HAVE_IMU_HEATER
#define HAL_HAVE_IMU_HEATER 0
#endif
