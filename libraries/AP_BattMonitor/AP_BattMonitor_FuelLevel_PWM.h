#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_FuelLevel_PWM : public AP_BattMonitor_Backend
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
    void irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp);

    struct IrqState {
        uint32_t last_pulse_us;
        uint32_t pulse_width_us;
        uint32_t pulse_count1;
    } irq_state;

    int8_t last_pin = -1;
    uint32_t pulse_count2;
};
