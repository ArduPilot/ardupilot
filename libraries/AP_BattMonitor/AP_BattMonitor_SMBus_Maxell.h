#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor_SMBus.h"
#include <AP_HAL/I2CDevice.h>

class AP_BattMonitor_SMBus_Maxell : public AP_BattMonitor_SMBus
{
public:

    // Constructor
    AP_BattMonitor_SMBus_Maxell(AP_BattMonitor &mon, uint8_t instance,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // read does nothing, all done in timer
    void read() override;

private:

    void timer(void);

    // read word from register
    // returns true if read was successful, false if failed
    bool read_word(uint8_t reg, uint16_t& data) const;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
