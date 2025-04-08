#pragma once

#include "AP_BattMonitor_Analog.h"

#if AP_BATTERY_FUELLEVEL_PWM_ENABLED

#include "AP_BattMonitor.h"

class AP_BattMonitor_FuelLevel_PWM : public AP_BattMonitor_Analog
{
public:

    /// Constructor
    AP_BattMonitor_FuelLevel_PWM(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return true; }

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    void init(void) override {}

private:

    AP_HAL::PWMSource pwm_source;
};

#endif  // AP_BATTERY_FUELLEVEL_PWM_ENABLED
