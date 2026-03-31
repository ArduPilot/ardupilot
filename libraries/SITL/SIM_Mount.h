/*
   Base class for simulated mount (gimbal) serial peripherals
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_MOUNT_ENABLED

#include "SIM_SerialDevice.h"

namespace SITL {

/*
  common base for all simulated mount backends.  Inherits SerialDevice
  so subclasses can be registered with create_serial_sim() and driven
  via a serial port.
*/
class Mount : public SerialDevice {
public:

    // called each simulation step
    virtual void update(const class Aircraft &aircraft) = 0;

    // called before add_gimbal_sim() so subclasses know their
    // 0-based instance index (used, e.g., to select a MAVLink component ID)
    virtual void set_instance(uint8_t instance) { _instance = instance; }

protected:
    uint8_t _instance;
};

}  // namespace SITL

#endif  // AP_SIM_MOUNT_ENABLED
