
#ifndef __AP_PROGMEM_H__
#define __AP_PROGMEM_H__

#include <AP_HAL_Boards.h>
#if defined(__AVR__) 
#include "AP_Progmem_AVR.h"
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL || CONFIG_HAL_BOARD == HAL_BOARD_SMACCM || CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
#include "AP_Progmem_Identity.h"
#else
#error "this build type is unknown - please edit AP_Progmem.h"
#endif

#define PROGMEM_STRING(_v, _s)  static const char _v[] PROGMEM = _s

#endif // __AP_PROGMEM_H__

