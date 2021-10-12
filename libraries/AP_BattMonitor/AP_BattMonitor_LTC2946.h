#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <utility>

#define HAL_BATTMON_LTC2946_ENABLED defined(HAL_BATTMON_LTC2946_BUS) && defined(HAL_BATTMON_LTC2946_ADDR)

#if HAL_BATTMON_LTC2946_ENABLED

class AP_BattMonitor_LTC2946 : public AP_BattMonitor_Backend
{
public:
    // inherit constructor
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    bool has_cell_voltages() const override { return false; }
    bool has_temperature() const override { return false; }
    bool has_current() const override { return true; }
    bool reset_remaining(float percentage) override { return false; }
    bool get_cycle_count(uint16_t &cycles) const override { return false; }

    virtual void init(void) override;
    virtual void read() override;
    
private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    bool read_word(const uint8_t reg, uint16_t& data) const;
    void timer(void);

    struct {
        uint16_t count;
        float volt_sum;
        float current_sum;
        HAL_Semaphore sem;
    } accumulate;
    float current_LSB;
    float voltage_LSB;
};

#endif // HAL_BATTMON_LTC2946_ENABLED
