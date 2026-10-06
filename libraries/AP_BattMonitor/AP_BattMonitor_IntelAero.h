#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_INTELAERO_ENABLED

#include "AP_BattMonitor_Backend.h"
#include <AP_HAL/I2CDevice.h>

/*
  battery voltage from the ADC in the Intel Aero RTF's FPGA, read over
  I2C by the Aero flight controller
 */
class AP_BattMonitor_IntelAero : public AP_BattMonitor_Backend
{
public:
    AP_BattMonitor_IntelAero(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

    void init(void) override;
    void read() override;

    bool has_current() const override { return false; }
    bool has_consumed_energy() const override { return false; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    void timer(void);

    AP_HAL::I2CDevice *dev;

    HAL_Semaphore sem;
    uint16_t raw_voltage;
    uint32_t last_sample_ms;

    AP_Float volt_multiplier;
};

#endif  // AP_BATTERY_INTELAERO_ENABLED
