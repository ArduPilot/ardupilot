#pragma once

#include "AP_BattMonitor_Analog.h"

#if AP_BATTERY_FUELFLOW_ENABLED

#include "AP_BattMonitor.h"

class AP_BattMonitor_FuelFlow : public AP_BattMonitor_Analog
{
public:

    /// Constructor
    AP_BattMonitor_FuelFlow(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return true; }

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    void init(void) override {}

private:
    void irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp);

    struct IrqState {
        uint32_t pulse_count;
        uint32_t total_us;
        uint32_t last_pulse_us;
    } irq_state;

    int8_t last_pin = -1;
};

#endif  // AP_BATTERY_FUELFLOW_ENABLED
