#include "AP_Frsky_Backend.h"

extern const AP_HAL::HAL& hal;

bool AP_Frsky_Backend::init()
{
    // if SPort Passthrough is using external data then it will
    // override this to do nothing:
    return init_serial_port();
}

bool AP_Frsky_Backend::init_serial_port()
{
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_Frsky_Backend::loop, void),
            "FrSky",
            1024,
            AP_HAL::Scheduler::PRIORITY_RCIN,
            1)) {
        return false;
    }
    // we don't want flow control for either protocol
    _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    return true;
}
