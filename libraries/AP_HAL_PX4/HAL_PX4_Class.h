
#ifndef __AP_HAL_PX4_CLASS_H__
#define __AP_HAL_PX4_CLASS_H__

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <AP_HAL_PX4.h>
#include "AP_HAL_PX4_Namespace.h"

class HAL_PX4 : public AP_HAL::HAL {
public:
    HAL_PX4();    
    void init(int argc, char * const argv[]) const;
};

extern const HAL_PX4 AP_HAL_PX4;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
#endif // __AP_HAL_PX4_CLASS_H__
