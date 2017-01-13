#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor_SMBus.h"
#include <AP_HAL/I2CDevice.h>

#define BATTMONITOR_SBUS_I2C_BUS 1
#define BATTMONITOR_SMBUS_I2C_ADDR 0x0B    // default I2C bus address

class AP_BattMonitor_SMBus_I2C : public AP_BattMonitor_SMBus
{
public:

    // Constructor
    AP_BattMonitor_SMBus_I2C(AP_BattMonitor &mon, uint8_t instance,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // Read the battery voltage and current.  Should be called at 10hz
    void read();

private:

    void timer(void);

    // read word from register
    // returns true if read was successful, false if failed
    bool read_word(uint8_t reg, uint16_t& data) const;

    // read_block - returns number of characters read if successful, zero if unsuccessful
    uint8_t read_block(uint8_t reg, uint8_t* data, uint8_t max_len, bool append_zero) const;

    // get_PEC - calculate PEC for a read or write from the battery
    //  buff is the data that was read or will be written
    uint8_t get_PEC(const uint8_t i2c_addr, uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len) const;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
