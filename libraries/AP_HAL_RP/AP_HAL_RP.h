#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include "HAL_RP_Class.h"

#if defined(HAL_MCU_RP2350A_ENABLED) && HAL_MCU_RP2350A_ENABLED == 1
    #include "rp2350a_gpio_defines.h"
#elif defined(HAL_MCU_RP2350B_ENABLED) && HAL_MCU_RP2350B_ENABLED == 1
    #include "rp2350b_gpio_defines.h"
#else
    #error "Unknown MCU type! Please define HAL_MCU_RP2350A_ENABLED or B in hwdef.dat"
#endif
