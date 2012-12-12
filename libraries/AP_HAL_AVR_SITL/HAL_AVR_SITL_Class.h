
#ifndef __AP_HAL_AVR_SITL_CLASS_H__
#define __AP_HAL_AVR_SITL_CLASS_H__

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

class HAL_AVR_SITL :: public AP_HAL::HAL {
    HAL_AVR_SITL();    
    void init(void*) const;
};

extern const HAL_AVR_SITL AP_HAL_AVR_SITL;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#endif // __AP_HAL_AVR_SITL_CLASS_H__

