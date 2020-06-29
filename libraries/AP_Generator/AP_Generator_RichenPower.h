// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include <stdint.h>
#include <stdio.h>

#ifndef GENERATOR_ENABLED
#define GENERATOR_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if GENERATOR_ENABLED

/*
 *  Example setup:
 *  param set SERIAL2_PROTOCOL 30
 *  param set SERIAL2_BAUD 9600
 *  param set RC9_OPTION 85
 *  param set SERVO8_FUNCTION 42
 */

class AP_Generator_RichenPower
{

public:

    AP_Generator_RichenPower();

    /* Do not allow copies */
    AP_Generator_RichenPower(const AP_Generator_RichenPower &other) = delete;
    AP_Generator_RichenPower &operator=(const AP_Generator_RichenPower&) = delete;

    static AP_Generator_RichenPower *get_singleton();

    void init();
    void update(void);

    void stop() { set_runstate(RunState::STOP); }
    void idle() { set_runstate(RunState::IDLE); }
    void run() { set_runstate(RunState::RUN); }

    void send_generator_status(const GCS_MAVLINK &channel);

    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const;

    // these return false if a reading is not available.  They do not
    // modify the passed-in value if they return false.
    bool voltage(float &voltage) const;
    bool current(float &current) const;

    bool healthy() const;

private:

    static AP_Generator_RichenPower *_singleton;

    bool get_reading();
    AP_HAL::UARTDriver *uart = nullptr;

    void update_servo_channel();
    uint32_t _last_servo_channel_check;
    void update_rc_channel();
    uint32_t _last_rc_channel_check;

    enum class RunState {
        STOP = 17,
        IDLE = 18,
        RUN = 19,
    };
    RunState runstate = RunState::STOP;
    void set_runstate(RunState newstate) {
        // gcs().send_text(MAV_SEVERITY_INFO, "RichenPower: Moving to state (%u) from (%u)\n", (unsigned)newstate, (unsigned)runstate);
        runstate = newstate;
    }
    void update_runstate();

    bool protocol_information_anounced;

    enum class Mode {
        IDLE = 0,
        RUN = 1,
        CHARGE = 2,
        BALANCE = 3,
    };

    struct Reading {
        uint32_t    runtime; //seconds
        uint32_t    seconds_until_maintenance;
        uint16_t    errors;
        uint16_t    rpm;
        float       output_voltage;
        float       output_current;
        Mode        mode;
    };

    void Log_Write();

    struct Reading last_reading;
    uint32_t last_reading_ms;

    const uint8_t HEADER_MAGIC1 = 0xAA;
    const uint8_t HEADER_MAGIC2 = 0x55;

    const uint8_t FOOTER_MAGIC1 = 0x55;
    const uint8_t FOOTER_MAGIC2 = 0xAA;

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

    struct PACKED RichenPacket {
        uint8_t headermagic1;
        uint8_t headermagic2;
        uint16_t version;
        uint8_t runtime_seconds;
        uint8_t runtime_minutes;
        uint16_t runtime_hours;
        uint32_t seconds_until_maintenance;
        uint16_t errors;
        uint16_t rpm;
        uint16_t throttle;
        uint16_t idle_throttle;
        uint16_t output_voltage;
        uint16_t output_current;
        uint16_t dynamo_current;
        uint8_t mode;
        uint8_t unknown1;
        uint8_t unknown6[38]; // "data"?!
        uint16_t checksum;
        uint8_t footermagic1;
        uint8_t footermagic2;
    };
    assert_storage_size<RichenPacket, 70> _assert_storage_size_RichenPacket;

    union RichenUnion {
        uint8_t parse_buffer[70];
        struct RichenPacket packet;
    };
    RichenUnion u;
    uint16_t *checksum_buffer = (uint16_t*)&u.parse_buffer[2];

    uint8_t body_length;

    // move the expected header bytes into &buffer[0], adjusting
    // body_length as appropriate.
    void move_header_in_buffer(uint8_t initial_offset);

    // servo channel used to control the generator:
    SRV_Channel *_servo_channel;

    // RC input generator for pilot to specify desired generator state
    RC_Channel *_rc_channel;

    static const uint16_t RUN_RPM = 15000;
    static const uint16_t IDLE_RPM = 4800;

    // logging state
    uint32_t last_logged_reading_ms;
};

namespace AP {
    AP_Generator_RichenPower *generator();
};

#endif
