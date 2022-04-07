#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"

class AP_BattMonitor_GP9301 : public AP_BattMonitor_Analog {
public:
    /// Constructor
    AP_BattMonitor_GP9301(
        AP_BattMonitor& mon, AP_BattMonitor::BattMonitor_State& mon_state, AP_BattMonitor_Params& params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return true; }

    /// returns true if battery monitor provides current info
    bool has_current() const override;

    void init(void) override { }

private:
    AP_HAL::PWMSource volt_pin_pwm_source;
    
    AP_HAL::PWMSource curr_pin_pwm_source;
};
