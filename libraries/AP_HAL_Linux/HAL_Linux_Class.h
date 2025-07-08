#pragma once

#include <AP_HAL/AP_HAL.h>

#include <signal.h>

class HAL_Linux : public AP_HAL::HAL {
public:
    HAL_Linux();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;

    void setup_signal_handlers() const;

    static void exit_signal_handler(int);

protected:
    volatile sig_atomic_t _should_exit = false;
};

#if HAL_NUM_CAN_IFACES
namespace Linux {
    class CANIface;
}
typedef Linux::CANIface HAL_CANIface;
#endif
