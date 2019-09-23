#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "SITL_State.h"

class HAL_SITL : public AP_HAL::HAL {
public:
    HAL_SITL();
    void run(int argc, char * const argv[], Callbacks* callbacks) const override;
    static void actually_reboot();

private:
    HALSITL::SITL_State *_sitl_state;

    void setup_signal_handlers() const;
    static void exit_signal_handler(int);
};

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL
