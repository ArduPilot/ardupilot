/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Arming {
public:
    enum ArmingChecks {
        ARMING_CHECK_NONE       = 0x0000,
        ARMING_CHECK_ALL        = 0x0001,
        ARMING_CHECK_BARO       = 0x0002,
        ARMING_CHECK_COMPASS    = 0x0004,
        ARMING_CHECK_GPS        = 0x0008,
        ARMING_CHECK_INS        = 0x0010,
        ARMING_CHECK_PARAMETERS = 0x0020,
        ARMING_CHECK_RC         = 0x0040,
        ARMING_CHECK_VOLTAGE    = 0x0080,
        ARMING_CHECK_BATTERY    = 0x0100,
        ARMING_CHECK_AIRSPEED   = 0x0200,
        ARMING_CHECK_LOGGING    = 0x0400,
        ARMING_CHECK_SWITCH     = 0x0800,
        ARMING_CHECK_GPS_CONFIG = 0x1000,
        ARMING_CHECK_RANGEFINDER_OPTFLOW = 0x2000,
        ARMING_CHECK_TERRAIN    = 0x4000,
        ARMING_CHECK_RALLYPOINT = 0x8000,
    };

    enum ArmingMethod {
        NONE = 0,
        RUDDER,
        MAVLINK
    };

    enum ArmingRequired {
        NO           = 0,
        YES_MIN_PWM  = 1,
        YES_ZERO_PWM = 2
    };

    enum ArmingRudder {
        ARMING_RUDDER_DISABLED  = 0,
        ARMING_RUDDER_ARMONLY   = 1,
        ARMING_RUDDER_ARMDISARM = 2
    };

    enum ArmingCheckResult {
        ARMING_CHECK_DISABLED = 0,
        ARMING_CHECK_FAILED = 1,
        ARMING_CHECK_PASSED = 2
    };

    struct FailureDescription {
        enum ArmingChecks group;
        const char *description;
    };

    const FailureDescription FailureDescriptions[2] = {
        {.group = ARMING_CHECK_BARO, .description = "Barometer not healthy"},
        {.group = ARMING_CHECK_BARO, .description = "Altitude disparity"}
    };

    AP_Arming(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass,
              const AP_BattMonitor &battery, const enum HomeState &home_set);

    ArmingRequired arming_required();
    bool arm(uint8_t method);
    bool disarm();
    bool is_armed();
    ArmingRudder rudder_arming() const { return (ArmingRudder)rudder_arming_value.get(); }
    uint16_t get_enabled_checks();

    /*
      pre_arm_checks() is virtual so it can be modified
      in a vehicle specific subclass
    */
    virtual bool pre_arm_checks(bool report);

    virtual ArmingCheckResult gps_checks(bool report);

    virtual ArmingCheckResult rangefinder_optflow_checks(bool report);

    void set_logging_available(bool set) { logging_available = set; }

    static const struct AP_Param::GroupInfo        var_info[];

protected:
    // Parameters
    AP_Int8                 require;
    AP_Int8                 rudder_arming_value;
    AP_Int16                checks_to_perform;      // bitmask for which checks are required
    AP_Float                accel_error_threshold;
    AP_Float                _min_voltage[AP_BATT_MONITOR_MAX_INSTANCES];

    // references
    const AP_AHRS           &ahrs;
    const AP_Baro           &barometer;
    Compass                 &_compass;
    const AP_BattMonitor    &_battery;
    const enum HomeState    &home_is_set;

    // internal members
    bool                    armed:1;
    bool                    logging_available:1;
    uint8_t                 arming_method;          // how the vehicle was armed
    uint32_t                last_accel_pass_ms[INS_MAX_INSTANCES];
    uint32_t                last_gyro_pass_ms[INS_MAX_INSTANCES];

    void set_enabled_checks(uint16_t);

    virtual ArmingCheckResult barometer_checks(bool report);

    ArmingCheckResult airspeed_checks(bool report);

    ArmingCheckResult logging_checks(bool report);

    virtual ArmingCheckResult ins_checks(bool report);

    virtual ArmingCheckResult parameter_checks(bool report);

    virtual ArmingCheckResult compass_checks(bool report);

    ArmingCheckResult battery_checks(bool report);

    ArmingCheckResult hardware_safety_check(bool report);

    ArmingCheckResult board_voltage_checks(bool report);

    virtual ArmingCheckResult manual_transmitter_checks(bool report);

    virtual ArmingCheckResult terrain_checks(bool report);
    
    virtual ArmingCheckResult rallypoint_checks(bool report);

    void update_enabled_passed_state(uint32_t check_id, ArmingCheckResult res, uint64_t &enabled_bitmask, uint64_t &passed_bitmask);
};
