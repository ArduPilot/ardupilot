#pragma once

#include "AP_Generator_config.h"

#if HAL_GENERATOR_ENABLED

#include <AP_Param/AP_Param.h>
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
    CLASS_NO_COPY(AP_Generator);

    static AP_Generator* get_singleton();

    void init(void);
    void update(void);

    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const;

    AP_BattMonitor::Failsafe update_failsafes(void);

    // Helpers to retrieve measurements
    float get_voltage(void) const { return _voltage; }
    float get_current(void) const { return _current; }
    // get_fuel_remaining returns fuel remaining as a scale 0-1
    float get_fuel_remaining(void) const { return _fuel_remaining; }
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

    void send_generator_status(const class GCS_MAVLINK &channel);

    // Parameter block
    static const struct AP_Param::GroupInfo var_info[];

    // bits which can be set in _options to modify generator behaviour:
    enum class Option {
        INHIBIT_MAINTENANCE_WARNINGS = 0,
    };

    bool option_set(Option opt) const {
        return (_options & 1U<<uint32_t(opt)) != 0;
    }

private:

    // Pointer to chosen driver
    AP_Generator_Backend* _driver_ptr;

    // Parameters
    AP_Int8 _type; // Select which generator to use
    AP_Int32 _options; // Select which generator to use

    enum class Type {
        GEN_DISABLED = 0,
#if AP_GENERATOR_IE650_800_ENABLED
        IE_650_800 = 1,
#endif
#if AP_GENERATOR_IE2400_ENABLED
        IE_2400 = 2,
#endif
#if AP_GENERATOR_RICHENPOWER_ENABLED
        RICHENPOWER = 3,
#endif
        // LOWEHEISER = 4,
    };

    // Helper to get param and cast to GenType
    Type type(void) const;

    // Front end variables
    float _voltage;
    float _current;
    float _fuel_remaining;  // 0-1
    bool _has_fuel_remaining;
    float _consumed_mah;
    uint16_t _rpm;
    bool _healthy;
    bool _has_current;
    bool _has_consumed_energy;

    static AP_Generator *_singleton;

};

namespace AP {
    AP_Generator *generator();
};
#endif
