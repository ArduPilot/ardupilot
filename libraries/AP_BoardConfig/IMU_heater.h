#pragma once

#include "IMU_heater_config.h"

#if HAL_HAVE_IMU_HEATER
#ifndef HAL_IMUHEAT_P_DEFAULT
#define HAL_IMUHEAT_P_DEFAULT 200
#endif
#ifndef HAL_IMUHEAT_I_DEFAULT
#define HAL_IMUHEAT_I_DEFAULT 0.3
#endif
#endif
