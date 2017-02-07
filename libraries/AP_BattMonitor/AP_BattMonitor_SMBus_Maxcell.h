#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor_SMBus.h"
#include <AP_HAL/I2CDevice.h>

class AP_BattMonitor_SMBus_Maxcell : public AP_BattMonitor_SMBus
{
public:

    // Constructor
    AP_BattMonitor_SMBus_Maxcell(AP_BattMonitor &mon, uint8_t instance,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // Read the battery voltage and current.  Should be called at 10hz
    void read();

private:

    void timer(void);

    // read word from register
    // returns true if read was successful, false if failed
    bool read_word(uint8_t reg, uint16_t& data, uint8_t size) const;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
