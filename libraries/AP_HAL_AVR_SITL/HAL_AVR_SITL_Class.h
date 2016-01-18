
#ifndef __AP_HAL_AVR_SITL_CLASS_H__
#define __AP_HAL_AVR_SITL_CLASS_H__

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_HAL_AVR_SITL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"
#include "SITL_State.h"

class HAL_AVR_SITL : public AP_HAL::HAL {
public:
    HAL_AVR_SITL();    
    void init(int argc, char * const argv[]) const;

private:
    AVR_SITL::SITL_State *_sitl_state;
};

extern const HAL_AVR_SITL AP_HAL_AVR_SITL;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#endif // __AP_HAL_AVR_SITL_CLASS_H__

