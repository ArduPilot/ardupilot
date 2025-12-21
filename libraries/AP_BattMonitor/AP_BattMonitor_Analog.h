#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_ANALOG_ENABLED

#include "AP_BattMonitor.h"

class AP_BattMonitor_Analog : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_Analog(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    virtual void read() override;

    /// returns true if battery monitor provides consumed energy info
    virtual bool has_consumed_energy() const override { return has_current(); }

    /// returns true if battery monitor provides current info
    virtual bool has_current() const override;

    virtual void init(void) override {}

    static const struct AP_Param::GroupInfo var_info[];

protected:

    AP_HAL::AnalogSource *_volt_pin_analog_source;
    AP_HAL::AnalogSource *_curr_pin_analog_source;

    // Parameters
    AP_Float _volt_multiplier;          /// voltage on volt pin multiplied by this to calculate battery voltage
    AP_Float _curr_amp_per_volt;        /// voltage on current pin multiplied by this to calculate current in amps
    AP_Float _curr_amp_offset;          /// offset voltage that is subtracted from current pin before conversion to amps
    AP_Float _volt_offset;              /// offset voltage that is subtracted from voltage pin before conversion
    AP_Int8  _volt_pin;                 /// board pin used to measure battery voltage
    AP_Int8  _curr_pin;                 /// board pin used to measure battery current
};

#endif  // AP_BATTERY_ANALOG_ENABLED
