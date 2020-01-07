#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <RC_Channel/RC_Channel.h>

class AP_Arming {
public:

    AP_Arming();

    /* Do not allow copies */
    AP_Arming(const AP_Arming &other) = delete;
    AP_Arming &operator=(const AP_Arming&) = delete;

    static AP_Arming *get_singleton();

    enum ArmingChecks {
        ARMING_CHECK_ALL         = (1U << 0),
        ARMING_CHECK_BARO        = (1U << 1),
        ARMING_CHECK_COMPASS     = (1U << 2),
        ARMING_CHECK_GPS         = (1U << 3),
        ARMING_CHECK_INS         = (1U << 4),
        ARMING_CHECK_PARAMETERS  = (1U << 5),
        ARMING_CHECK_RC          = (1U << 6),
        ARMING_CHECK_VOLTAGE     = (1U << 7),
        ARMING_CHECK_BATTERY     = (1U << 8),
        ARMING_CHECK_AIRSPEED    = (1U << 9),
        ARMING_CHECK_LOGGING     = (1U << 10),
        ARMING_CHECK_SWITCH      = (1U << 11),
        ARMING_CHECK_GPS_CONFIG  = (1U << 12),
        ARMING_CHECK_SYSTEM      = (1U << 13),
        ARMING_CHECK_MISSION     = (1U << 14),
        ARMING_CHECK_RANGEFINDER = (1U << 15),
        ARMING_CHECK_CAMERA      = (1U << 16),
    };

    enum class Method {
        RUDDER = 0,
        MAVLINK = 1,
        AUXSWITCH = 2,
        MOTORTEST = 3,
        SCRIPTING = 4,
    };

    enum class Required {
        NO           = 0,
        YES_MIN_PWM  = 1,
        YES_ZERO_PWM = 2
    };

    void init(void);

    // these functions should not be used by Copter which holds the armed state in the motors library
    Required arming_required();
    virtual bool arm(AP_Arming::Method method, bool do_arming_checks=true);
    virtual bool disarm();
    bool is_armed();

    // get bitmask of enabled checks
    uint16_t get_enabled_checks();

    // pre_arm_checks() is virtual so it can be modified in a vehicle specific subclass
    virtual bool pre_arm_checks(bool report);

    // some arming checks have side-effects, or require some form of state
    // change to have occurred, and thus should not be done as pre-arm
    // checks.  Those go here:
    virtual bool arm_checks(AP_Arming::Method method);

    // get expected magnetic field strength
    uint16_t compass_magfield_expected() const;

    // rudder arming support
    enum class RudderArming {
        IS_DISABLED  = 0, // DISABLED leaks in from vehicle defines.h
        ARMONLY   = 1,
        ARMDISARM = 2
    };

    RudderArming get_rudder_arming_type() const { return (RudderArming)_rudder_arming.get(); }

    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // Parameters
    AP_Int8                 require;
    AP_Int32                checks_to_perform;      // bitmask for which checks are required
    AP_Float                accel_error_threshold;
    AP_Int8                 _rudder_arming;
    AP_Int32                 _required_mission_items;

    // internal members
    bool                    armed;
    uint32_t                last_accel_pass_ms[INS_MAX_INSTANCES];
    uint32_t                last_gyro_pass_ms[INS_MAX_INSTANCES];

    virtual bool barometer_checks(bool report);

    bool airspeed_checks(bool report);

    bool logging_checks(bool report);

    virtual bool ins_checks(bool report);

    virtual bool compass_checks(bool report);

    virtual bool gps_checks(bool report);

    bool battery_checks(bool report);

    bool hardware_safety_check(bool report);

    virtual bool board_voltage_checks(bool report);

    virtual bool rc_calibration_checks(bool report);

    bool manual_transmitter_checks(bool report);

    bool mission_checks(bool report);

    bool rangefinder_checks(bool report);

    bool fence_checks(bool report);

    bool camera_checks(bool display_failure);

    virtual bool system_checks(bool report);

    bool can_checks(bool report);

    virtual bool proximity_checks(bool report) const;

    bool servo_checks(bool report) const;
    bool rc_checks_copter_sub(bool display_failure, const RC_Channel *channels[4]) const;

    // mandatory checks that cannot be bypassed.  This function will only be called if ARMING_CHECK is zero or arming forced
    virtual bool mandatory_checks(bool report) { return true; }

    // returns true if a particular check is enabled
    bool check_enabled(const enum AP_Arming::ArmingChecks check) const;
    // returns a mavlink severity which should be used if a specific check fails
    MAV_SEVERITY check_severity(const enum AP_Arming::ArmingChecks check) const;
    // handle the case where a check fails
    void check_failed(const enum AP_Arming::ArmingChecks check, bool report, const char *fmt, ...) const FMT_PRINTF(4, 5);
    void check_failed(bool report, const char *fmt, ...) const FMT_PRINTF(3, 4);

    void Log_Write_Arm(bool forced, AP_Arming::Method method);
    void Log_Write_Disarm();

private:

    static AP_Arming *_singleton;

    bool ins_accels_consistent(const AP_InertialSensor &ins);
    bool ins_gyros_consistent(const AP_InertialSensor &ins);

    enum MIS_ITEM_CHECK {
        MIS_ITEM_CHECK_LAND          = (1 << 0),
        MIS_ITEM_CHECK_VTOL_LAND     = (1 << 1),
        MIS_ITEM_CHECK_DO_LAND_START = (1 << 2),
        MIS_ITEM_CHECK_TAKEOFF       = (1 << 3),
        MIS_ITEM_CHECK_VTOL_TAKEOFF  = (1 << 4),
        MIS_ITEM_CHECK_RALLY         = (1 << 5),
        MIS_ITEM_CHECK_MAX
    };
};

namespace AP {
    AP_Arming &arming();
};
