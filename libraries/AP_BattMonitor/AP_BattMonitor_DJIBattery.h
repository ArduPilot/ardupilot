#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_DJIBattery : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_DJIBattery(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and percentage.
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return true; }

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    void init(void) override {}

private:
    AP_HAL::UARTDriver *port;
    void irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp);
    void DecToHex(int dec);
    
    struct IrqState {
        uint32_t last_pulse_us;
        uint32_t pulse_width_us;
        uint32_t pulse_count1;
    } irq_state;

    int8_t last_pin = -1;
    uint32_t pulse_count2;
    
    uint32_t last_send_ms;
    uint32_t delay_time_ms;
    uint32_t Count;
    uint8_t pktbuf[64];
    char hex[2];
    bool _PortAvailable;
    double _Volt;
    double _Volt1;
    double _Volt2;
    double _Volt3;
    double _Volt4;
    double _Percentage;
    double _UsedCapacity;

protected:

    AP_HAL::AnalogSource *_volt_pin_analog_source;
};