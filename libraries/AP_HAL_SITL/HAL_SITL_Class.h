
#ifndef __AP_HAL_SITL_CLASS_H__
#define __AP_HAL_SITL_CLASS_H__

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "SITL_State.h"

class HAL_SITL : public AP_HAL::HAL {
public:
    HAL_SITL();
    void run(int argc, char * const argv[], Callbacks* callbacks) const override;

private:
    HALSITL::SITL_State *_sitl_state;
};

extern const HAL_SITL AP_HAL_SITL;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
#endif // __AP_HAL_SITL_CLASS_H__

