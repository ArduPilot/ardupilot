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

    void set_storage_posix_enabled(bool _enabled) {
        storage_posix_enabled = _enabled;
    }
    bool get_storage_posix_enabled() const { return storage_posix_enabled; }
    void set_storage_flash_enabled(bool _enabled) {
        storage_flash_enabled = _enabled;
    }
    bool get_storage_flash_enabled() const { return storage_flash_enabled; }

private:
    HALSITL::SITL_State *_sitl_state;

    void setup_signal_handlers() const;
    static void exit_signal_handler(int);

    bool storage_posix_enabled;
    bool storage_flash_enabled;

};

#if HAL_NUM_CAN_IFACES
typedef HALSITL::CANIface HAL_CANIface;
#endif

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL
