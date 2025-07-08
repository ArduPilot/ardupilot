// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Generator_config.h"

#if AP_GENERATOR_RICHENPOWER_ENABLED

#include "AP_Generator_Backend.h"

#include <AP_Logger/AP_Logger_config.h>
#include <AP_Common/AP_Common.h>
#include <stdint.h>
#include <stdio.h>


/*
 *  Example setup:
 *  param set SERIAL2_PROTOCOL 30  # Generator protocol
 *  param set SERIAL2_BAUD 9600
 *  param set RC9_OPTION 85        # pilot directive for generator
 *  param set SERVO8_FUNCTION 42   # autopilot directive to generator
 */

class AP_Generator_RichenPower : public AP_Generator_Backend
{

public:
    // constructor
    using AP_Generator_Backend::AP_Generator_Backend;

    // init should be called at vehicle startup to get the generator library ready
    __INITFUNC__ void init(void) override;
    // update should be called regularly to update the generator state
    void update(void) override;

    // methods to control the generator state:
    bool stop(void) override;
    bool idle(void) override;
    bool run(void) override;

    // method to send a GENERATOR_STATUS mavlink message
    void send_generator_status(const GCS_MAVLINK &channel) override;

    // prearm checks to ensure generator is good for arming.  Note
    // that if the generator has never sent us a message then these
    // automatically pass!
    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const override;

    // Update front end with voltage, current, and rpm values
    void update_frontend_readings(void);

    // healthy returns true if the generator is not present, or it is
    // present, providing telemetry and not indicating an errors.
    bool healthy() const override;

private:

    // read - read serial port, return true if a new reading has been found
    bool get_reading();
    AP_HAL::UARTDriver *uart;

    // methods and state to record pilot desired runstate and actual runstate:
    enum class RunState {
        STOP = 17,
        IDLE = 18,
        RUN = 19,
    };
    RunState pilot_desired_runstate = RunState::STOP;
    RunState commanded_runstate = RunState::STOP;  // output is based on this
    void set_pilot_desired_runstate(RunState newstate) {
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RichenPower: Moving to state (%u) from (%u)", (unsigned)newstate, (unsigned)runstate);
        pilot_desired_runstate = newstate;
    }
    void update_runstate();

    // boolean so we can emit the RichenPower protocol version once
    bool protocol_information_anounced;

    // reported mode from the generator:
    enum class Mode {
        IDLE = 0,
        RUN = 1,
        CHARGE = 2,
        BALANCE = 3,
        OFF = 4,
    };

    // un-packed data from the generator:
    struct Reading {
        uint32_t    runtime; //seconds
        uint32_t    seconds_until_maintenance;
        uint16_t    errors;
        uint16_t    rpm;
        float       output_voltage;
        float       output_current;
        Mode        mode;
    };

#if HAL_LOGGING_ENABLED
    // method and state to write and entry to the onboard log:
    void Log_Write();
    uint32_t last_logged_reading_ms;
#endif

    struct Reading last_reading;
    uint32_t last_reading_ms;

    const uint8_t HEADER_MAGIC1 = 0xAA;
    const uint8_t HEADER_MAGIC2 = 0x55;

    const uint8_t FOOTER_MAGIC1 = 0x55;
    const uint8_t FOOTER_MAGIC2 = 0xAA;

    // reported errors from the generator:
    enum class Errors { // bitmask
        MaintenanceRequired = 0,
        StartDisabled = 1,
        Overload = 2,
        LowVoltageOutput = 3,
        LowBatteryVoltage = 4,
        LAST
    };

    const char *error_strings[5] = {
        "MaintenanceRequired",
        "StartDisabled",
        "Overload",
        "LowVoltageOutput",
        "LowBatteryVoltage",
    };
    static_assert(ARRAY_SIZE(error_strings) == (uint8_t)Errors::LAST,
                  "have error string for each error");

    // RichenPower data packet format:
    struct PACKED RichenPacket {
        uint8_t headermagic1;
        uint8_t headermagic2;
        uint16_t version;
        uint8_t runtime_minutes;
        uint8_t runtime_seconds;
        uint16_t runtime_hours;
        uint16_t seconds_until_maintenance_high;
        uint16_t seconds_until_maintenance_low;
        uint16_t errors;
        uint16_t rpm;
        uint16_t throttle;
        uint16_t idle_throttle;
        uint16_t output_voltage;
        uint16_t output_current;
        uint16_t dynamo_current;
        uint8_t unknown1;
        uint8_t mode;
        uint8_t unknown6[38]; // "data"?!
        uint16_t checksum;
        uint8_t footermagic1;
        uint8_t footermagic2;
    };

    union RichenUnion {
        uint8_t parse_buffer[70];
        struct RichenPacket packet;
    };
    RichenUnion u;

    // number of bytes currently in the buffer
    uint8_t body_length;

    // move the expected header bytes into &buffer[0], adjusting
    // body_length as appropriate.
    void move_header_in_buffer(uint8_t initial_offset);

    // a simple heat model to avoid the motor moving to run too fast
    // or being stopped before cooldown.  The generator itself does
    // not supply temperature via telemetry, so we fake one based on
    // RPM.
    uint32_t last_heat_update_ms;
    float heat;
    void update_heat();

    // returns true if the generator should be allowed to move into
    // the "run" (high-RPM) state:
    bool generator_ok_to_run() const;
    // returns an amount of synthetic heat required for the generator
    // to move into the "run" state:
    static constexpr float heat_required_for_run();

    // approximate run and idle speeds for the generator:
    static const uint16_t RUN_RPM = 15000;
    static const uint16_t IDLE_RPM = 6000;

    static constexpr float heat_environment_loss_factor = 0.005f;
    // powf is not constexpr, so we create a const for it:
    // powf(1.0f-heat_environment_loss_factor, 30)
    static constexpr float heat_environment_loss_30s = 0.860384;
    static constexpr float heat_environment_loss_60s = 0.740261;

    // boolean so we can announce we've stopped the generator due to a
    // crash just once:
    bool vehicle_was_crashed;

    // data and methods to handle time-in-idle-state:
    uint32_t idle_state_start_ms;

    uint32_t time_in_idle_state_ms() const {
        if (idle_state_start_ms == 0) {
            return 0;
        }
        return AP_HAL::millis() - idle_state_start_ms;
    }

    // check if the generator requires maintenance and send a message if it does:
    void check_maintenance_required();
    // if we are emitting warnings about the generator requiring
    // maintenamce, this is the last time we sent the warning:
    uint32_t last_maintenance_warning_ms;
};
#endif  // AP_GENERATOR_RICHENPOWER_ENABLED
