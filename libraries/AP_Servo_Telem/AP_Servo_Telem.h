#pragma once

#include "AP_Servo_Telem_config.h"

#if AP_SERVO_TELEM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel_config.h>


#ifndef SERVO_TELEM_MAX_SERVOS
    #define SERVO_TELEM_MAX_SERVOS NUM_SERVO_CHANNELS
#endif
static_assert(SERVO_TELEM_MAX_SERVOS > 0, "Cannot have 0 Servo telem instances");

class AP_Servo_Telem {
public:
    AP_Servo_Telem();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Servo_Telem);

    static AP_Servo_Telem *get_singleton();

    struct TelemetryData {
        // Telemetry values
        float command_position;         // Commanded servo position in degrees
        float measured_position;        // Measured Servo position in degrees
        float force;                    // Force in Newton meters
        float speed;                    // Speed degrees per second
        float voltage;                  // Voltage in volts
        float current;                  // Current draw in Ampere
        uint8_t duty_cycle;             // duty cycle 0% to 100%
        int16_t motor_temperature_cdeg; // centi-degrees C, negative values allowed
        int16_t pcb_temperature_cdeg;   // centi-degrees C, negative values allowed
        uint8_t status_flags;           // Type specific status flags

        // last update time in milliseconds, determines data is stale
        uint32_t last_update_ms;

        // telemetry types present
        enum Types {
            COMMANDED_POSITION = 1 << 0,
            MEASURED_POSITION  = 1 << 1,
            FORCE              = 1 << 2,
            SPEED              = 1 << 3,
            VOLTAGE            = 1 << 4,
            CURRENT            = 1 << 5,
            DUTY_CYCLE         = 1 << 6,
            MOTOR_TEMP         = 1 << 7,
            PCB_TEMP           = 1 << 8,
            STATUS             = 1 << 9,
        };
        uint16_t present_types;

        // return true if the requested types of data are available
        bool present(const uint16_t type_mask) const volatile;
    };

    // update at 10Hz to log telemetry
    void update();

    // record an update to the telemetry data together with timestamp
    // callback to update the data in the frontend, should be called by the driver when new data is available
    void update_telem_data(const uint8_t servo_index, const TelemetryData& new_data);

    // Fill in telem structure if telem is available, return false if not
    bool get_telem(const uint8_t servo_index, TelemetryData& telem) const volatile;

private:

    // Log telem of each servo
    void write_log();

    volatile TelemetryData _telem_data[SERVO_TELEM_MAX_SERVOS];

    uint32_t _last_telem_log_ms[SERVO_TELEM_MAX_SERVOS];

    static AP_Servo_Telem *_singleton;

    uint32_t active_mask;
};
#endif // AP_SERVO_TELEM_ENABLED
