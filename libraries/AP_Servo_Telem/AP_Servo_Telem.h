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
        uint16_t valid_types;

        // return true if the data is stale
        bool stale(uint32_t now_ms) const volatile;

        // return true if the requested types of data are available
        bool present(const uint16_t type_mask) const volatile;

        //  return true if the requested type of data is available and not stale
        bool valid(const uint16_t type_mask) const volatile;
    };

    // update at 10Hz to log telemetry
    void update();

    // record an update to the telemetry data together with timestamp
    // callback to update the data in the frontend, should be called by the driver when new data is available
    void update_telem_data(const uint8_t servo_index, const TelemetryData& new_data);

    // Getters for telem values

    // Return the commanded position servo position in degrees if available
    bool get_commanded_position(const uint8_t servo_index, float &command_position) const;

    // Return the measured Servo position in degrees if available
    bool get_measured_position(const uint8_t servo_index, float &measured_position) const;

    // Return the force in newton meters if available
    bool get_force(const uint8_t servo_index, float &force) const;

    // Return the speed in degrees per second if available
    bool get_speed(const uint8_t servo_index, float &speed) const;

    // Return the voltage in volts per second if available
    bool get_voltage(const uint8_t servo_index, float &voltage) const;

    // Return the current in amps per second if available
    bool get_current(const uint8_t servo_index, float &current) const;

    // Return the duty cycle 0% to 100% if available
    bool get_duty_cycle(const uint8_t servo_index, uint8_t &duty_cycle) const;

    // Return the motor temperature in degrees C if available
    bool get_motor_temperature(const uint8_t servo_index, float &motor_temperature) const;

    // Return the pcb temperature in degrees C if available
    bool get_pcb_temperature(const uint8_t servo_index, float &pcb_temperature) const;

    // Return type specific status flags if available
    bool get_status_flags(const uint8_t servo_index, uint8_t &status_flags) const;

private:

    // Helper to check index and if data is available
    bool data_available(const uint8_t servo_index, const TelemetryData::Types type) const;

    // Log telem of each servo
    void write_log();

    volatile TelemetryData _telem_data[SERVO_TELEM_MAX_SERVOS];

    uint32_t _last_telem_log_ms[SERVO_TELEM_MAX_SERVOS];

    static AP_Servo_Telem *_singleton;

    uint32_t active_mask;
};
#endif // AP_SERVO_TELEM_ENABLED
