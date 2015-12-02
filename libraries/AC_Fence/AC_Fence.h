/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_FENCE_H
#define AC_FENCE_H

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library

// bit masks for enabled fence types.  Used for TYPE parameter
#define AC_FENCE_TYPE_NONE                          0       // fence disabled
#define AC_FENCE_TYPE_ALT_MAX                       1       // high alt fence which usually initiates an RTL
#define AC_FENCE_TYPE_CIRCLE                        2       // circular horizontal fence (usually initiates an RTL)

// valid actions should a fence be breached
#define AC_FENCE_ACTION_REPORT_ONLY                 0       // report to GCS that boundary has been breached but take no further action
#define AC_FENCE_ACTION_RTL_AND_LAND                1       // return to launch and, if that fails, land

// default boundaries
#define AC_FENCE_ALT_MAX_DEFAULT                    100.0f  // default max altitude is 100m
#define AC_FENCE_CIRCLE_RADIUS_DEFAULT              300.0f  // default circular fence radius is 300m
#define AC_FENCE_ALT_MAX_BACKUP_DISTANCE            20.0f   // after fence is broken we recreate the fence 20m further up
#define AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE      20.0f   // after fence is broken we recreate the fence 20m further out
#define AC_FENCE_MARGIN_DEFAULT                     2.0f    // default distance in meters that autopilot's should maintain from the fence to avoid a breach

// give up distance
#define AC_FENCE_GIVE_UP_DISTANCE                   100.0f  // distance outside the fence at which we should give up and just land.  Note: this is not used by library directly but is intended to be used by the main code
#define AC_FENCE_MANUAL_RECOVERY_TIME_MIN           10000   // pilot has 10seconds to recover during which time the autopilot will not attempt to re-take control

class AC_Fence
{
public:

    /// Constructor
    AC_Fence(const AP_InertialNav& inav);

    /// enable - allows fence to be enabled/disabled.  Note: this does not update the eeprom saved value
    void enable(bool true_false) { _enabled = true_false; }

    /// enabled - returns true if fence is enabled
    bool enabled() const { return _enabled; }

    /// get_enabled_fences - returns bitmask of enabled fences
    uint8_t get_enabled_fences() const;

    /// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
    bool pre_arm_check() const;

    ///
    /// methods to check we are within the boundaries and recover
    ///

    /// check_fence - returns the fence type that has been breached (if any)
    ///     curr_alt is the altitude above home in meters
    uint8_t check_fence(float curr_alt);

    /// get_breaches - returns bit mask of the fence types that have been breached
    uint8_t get_breaches() const { return _breached_fences; }

    /// get_breach_time - returns time the fence was breached
    uint32_t get_breach_time() const { return _breach_time; }

    /// get_breach_count - returns number of times we have breached the fence
    uint16_t get_breach_count() const { return _breach_count; }

    /// get_breach_distance - returns distance in meters outside of the given fence
    float get_breach_distance(uint8_t fence_type) const;

    /// get_action - getter for user requested action on limit breach
    uint8_t get_action() const { return _action.get(); }

    /// get_safe_alt - returns maximum safe altitude (i.e. alt_max - margin)
    float get_safe_alt() const { return _alt_max - _margin; }

    /// manual_recovery_start - caller indicates that pilot is re-taking manual control so fence should be disabled for 10 seconds
    ///     should be called whenever the pilot changes the flight mode
    ///     has no effect if no breaches have occurred
    void manual_recovery_start();

    ///
    /// time saving methods to piggy-back on main code's calculations
    ///

    /// set_home_distance - update vehicle's distance from home in meters - required for circular horizontal fence monitoring
    void set_home_distance(float distance) { _home_distance = distance; }

    static const struct AP_Param::GroupInfo var_info[];

private:

    /// record_breach - update breach bitmask, time and count
    void record_breach(uint8_t fence_type);

    /// clear_breach - update breach bitmask, time and count
    void clear_breach(uint8_t fence_type);

    // pointers to other objects we depend upon
    const AP_InertialNav& _inav;

    // parameters
    AP_Int8         _enabled;               // top level enable/disable control
    AP_Int8         _enabled_fences;        // bit mask holding which fences are enabled
    AP_Int8         _action;                // recovery action specified by user
    AP_Float        _alt_max;               // altitude upper limit in meters
    AP_Float        _circle_radius;         // circle fence radius in meters
    AP_Float        _margin;                // distance in meters that autopilot's should maintain from the fence to avoid a breach

    // backup fences
    float           _alt_max_backup;        // backup altitude upper limit in meters used to refire the breach if the vehicle continues to move further away
    float           _circle_radius_backup;  // backup circle fence radius in meters used to refire the breach if the vehicle continues to move further away

    // breach distances
    float           _alt_max_breach_distance;   // distance above the altitude max
    float           _circle_breach_distance;    // distance beyond the circular fence

    // other internal variables
    float           _home_distance;         // distance from home in meters (provided by main code)

    // breach information
    uint8_t         _breached_fences;       // bitmask holding the fence type that was breached (i.e. AC_FENCE_TYPE_ALT_MIN, AC_FENCE_TYPE_CIRCLE)
    uint32_t        _breach_time;           // time of last breach in milliseconds
    uint16_t        _breach_count;          // number of times we have breached the fence

    uint32_t        _manual_recovery_start_ms;  // system time in milliseconds that pilot re-took manual control
};
#endif	// AC_FENCE_H
