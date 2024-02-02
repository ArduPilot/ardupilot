#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "AP_HAL_Boards.h"

class AP_HAL::AnalogSource {
public:
    virtual float read_average() = 0;
    virtual float read_latest() = 0;
    virtual bool set_pin(uint8_t p) WARN_IF_UNUSED = 0;

    // return a voltage from 0.0 to 5.0V, scaled
    // against a reference voltage
    virtual float voltage_average() = 0;

    // return a voltage from 0.0 to 5.0V, scaled
    // against a reference voltage
    virtual float voltage_latest() = 0;

    // return a voltage from 0.0 to 5.0V, assuming a ratiometric
    // sensor
    virtual float voltage_average_ratiometric() = 0;
};

class AP_HAL::AnalogIn {
public:
    virtual void init() = 0;
    virtual AP_HAL::AnalogSource* channel(int16_t n) = 0;

    // board 5V rail voltage in volts
    virtual float board_voltage(void) = 0;

    // servo rail voltage in volts, or 0 if unknown
    virtual float servorail_voltage(void) { return 0; }

    // power supply status flags, see MAV_POWER_STATUS
    virtual uint16_t power_status_flags(void) { return 0; }

    // bitmask of all _power_flags bits ever set, so transient
    // failures can still be diagnosed
    virtual uint16_t accumulated_power_status_flags(void) const { return 0; }

    // this enum class is 1:1 with MAVLink's MAV_POWER_STATUS enumeration!
    enum class PowerStatusFlag : uint16_t {
        BRICK_VALID = 1,                  // main brick power supply valid
        SERVO_VALID = 2,                  // main servo power supply valid for FMU
        USB_CONNECTED = 4,                // USB power is connected
        PERIPH_OVERCURRENT = 8,           // peripheral supply is in over-current state
        PERIPH_HIPOWER_OVERCURRENT = 16,  // hi-power peripheral supply is in over-current state
        CHANGED = 32,                     // Power status has changed since boot
    };

#if HAL_WITH_MCU_MONITORING
    virtual float mcu_temperature(void) { return 0; }
    virtual float mcu_voltage(void) { return 0; }
    virtual float mcu_voltage_max(void) { return 0; }
    virtual float mcu_voltage_min(void) { return 0; }
#endif
};

#define ANALOG_INPUT_BOARD_VCC 254
#define ANALOG_INPUT_NONE 255
