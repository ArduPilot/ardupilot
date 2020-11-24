#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef GENERATOR_ENABLED
#define GENERATOR_ENABLED !HAL_MINIMIZE_FEATURES && !defined(HAL_BUILD_AP_PERIPH)
#endif

#if GENERATOR_ENABLED

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

class AP_Generator_Backend;
class AP_Generator_IE_650_800;
class AP_Generator_IE_2400;
class AP_Generator_RichenPower;

class AP_Generator
{
    friend class AP_Generator_Backend;
    friend class AP_Generator_IE_650_800;
    friend class AP_Generator_IE_2400;
    friend class AP_Generator_RichenPower;

public:
    // Constructor
    AP_Generator();

    // Do not allow copies
    AP_Generator(const AP_Generator &other) = delete;
    AP_Generator &operator=(const AP_Generator&) = delete;

    static AP_Generator* get_singleton();

    void init(void);
    void update(void);

    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const;

    AP_BattMonitor::Failsafe update_failsafes(void);

    // Helpers to retrieve measurements
    float get_voltage(void) const { return _voltage; }
    float get_current(void) const { return _current; }
    float get_fuel_remain(void) const { return _fuel_remain_pct; }
    float get_batt_consumed(void) const { return _consumed_mah; }
    uint16_t get_rpm(void) const { return _rpm; }

    // Helpers to see if backend has a measurement
    bool has_current() const { return _has_current; }
    bool has_consumed_energy() const { return _has_consumed_energy; }
    bool has_fuel_remaining() const { return _has_fuel_remaining; }

    // healthy() returns true if the generator is not present, or it is
    // present, providing telemetry and not indicating any errors.
    bool healthy(void) const { return _healthy; }

    // Generator controls must return true if present in generator type
    bool stop(void);
    bool idle(void);
    bool run(void);

    void send_generator_status(const GCS_MAVLINK &channel);

    // Parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    // Pointer to chosen driver
    AP_Generator_Backend* _driver_ptr;

    // Parameters
    AP_Int8 _type; // Select which generator to use

    enum class Type {
        GEN_DISABLED = 0,
        IE_650_800 = 1,
        IE_2400 = 2,
        RICHENPOWER = 3,
    };

    // Helper to get param and cast to GenType
    Type type(void) const;

    // Front end variables
    float _voltage;
    float _current;
    float _fuel_remain_pct;
    float _consumed_mah;
    uint16_t _rpm;
    bool _healthy;
    bool _has_current;
    bool _has_consumed_energy;
    bool _has_fuel_remaining;

    static AP_Generator *_singleton;

};

namespace AP {
    AP_Generator *generator();
};
#endif
