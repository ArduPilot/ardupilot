
#ifndef __AP_HAL_AVR_SITL_CLASS_H__
#define __AP_HAL_AVR_SITL_CLASS_H__

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_HAL_AVR_SITL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"

class HAL_AVR_SITL : public AP_HAL::HAL {
public:
    HAL_AVR_SITL();    
    void init(int argc, const char *argv[]) const;
};

extern const HAL_AVR_SITL AP_HAL_AVR_SITL;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#endif // __AP_HAL_AVR_SITL_CLASS_H__

