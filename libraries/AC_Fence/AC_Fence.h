#pragma once

#include "AC_Fence_config.h"

#if AP_FENCE_ENABLED

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_Fence/AC_PolyFence_loader.h>

// bit masks for enabled fence types.  Used for TYPE parameter
#define AC_FENCE_TYPE_ALT_MAX                       1       // high alt fence which usually initiates an RTL
#define AC_FENCE_TYPE_CIRCLE                        2       // circular horizontal fence (usually initiates an RTL)
#define AC_FENCE_TYPE_POLYGON                       4       // polygon horizontal fence
#define AC_FENCE_TYPE_ALT_MIN                       8       // low alt fence which usually initiates an RTL

// valid actions should a fence be breached
#define AC_FENCE_ACTION_REPORT_ONLY                 0       // report to GCS that boundary has been breached but take no further action
#define AC_FENCE_ACTION_RTL_AND_LAND                1       // return to launch and, if that fails, land
#define AC_FENCE_ACTION_ALWAYS_LAND                 2       // always land
#define AC_FENCE_ACTION_SMART_RTL                   3       // smartRTL, if that fails, RTL, it that still fails, land
#define AC_FENCE_ACTION_BRAKE                       4       // brake, if that fails, land
#define AC_FENCE_ACTION_SMART_RTL_OR_LAND           5       // SmartRTL, if that fails, Land
#define AC_FENCE_ACTION_GUIDED                      6       // guided mode, with target waypoint as fence return point
#define AC_FENCE_ACTION_GUIDED_THROTTLE_PASS        7       // guided mode, but pilot retains manual throttle control

// default boundaries
#define AC_FENCE_ALT_MAX_DEFAULT                    100.0f  // default max altitude is 100m
#define AC_FENCE_ALT_MIN_DEFAULT                    -10.0f  // default maximum depth in meters
#define AC_FENCE_CIRCLE_RADIUS_DEFAULT              300.0f  // default circular fence radius is 300m
#define AC_FENCE_ALT_MAX_BACKUP_DISTANCE            20.0f   // after fence is broken we recreate the fence 20m further up
#define AC_FENCE_ALT_MIN_BACKUP_DISTANCE            20.0f   // after fence is broken we recreate the fence 20m further down
#define AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE      20.0f   // after fence is broken we recreate the fence 20m further out
#define AC_FENCE_MARGIN_DEFAULT                     2.0f    // default distance in meters that autopilot's should maintain from the fence to avoid a breach

// give up distance
#define AC_FENCE_GIVE_UP_DISTANCE                   100.0f  // distance outside the fence at which we should give up and just land.  Note: this is not used by library directly but is intended to be used by the main code
#define AC_FENCE_MANUAL_RECOVERY_TIME_MIN           10000   // pilot has 10seconds to recover during which time the autopilot will not attempt to re-take control

class AC_Fence
{
public:

    enum class AutoEnable
    {
        ALWAYS_DISABLED = 0,
        ALWAYS_ENABLED = 1,
        ENABLE_DISABLE_FLOOR_ONLY = 2,
        ONLY_WHEN_ARMED = 3
    };

    AC_Fence();

    /* Do not allow copies */
    AC_Fence(const AC_Fence &other) = delete;
    AC_Fence &operator=(const AC_Fence&) = delete;

    void init() {
        _poly_loader.init();
    }

    // get singleton instance
    static AC_Fence *get_singleton() { return _singleton; }

    /// enable - allows fence to be enabled/disabled.
    void enable(bool value);

    /// auto_enabled - automaticaly enable/disable fence depending of flight status
    AutoEnable auto_enabled() { return static_cast<AutoEnable>(_auto_enabled.get()); }

    /// enable_floor - allows fence floor to be enabled/disabled. Note this does not update the eeprom saved value
    void enable_floor();

    /// disable_floor - allows fence floor to be enabled/disabled. Note this does not update the eeprom saved value
    void disable_floor();

    /// auto_enable_fence_on_takeoff - auto enables the fence. Called after takeoff conditions met
    void auto_enable_fence_after_takeoff();

    /// auto_disable_fence_for_landing - auto disables respective fence. Called prior to landing.
    void auto_disable_fence_for_landing();

    /// enabled - returns true if fence is enabled
    bool enabled() const { return _enabled; }

    /// present - returns true if fence is present
    bool present() const;

    /// get_enabled_fences - returns bitmask of enabled fences
    uint8_t get_enabled_fences() const;

    // should be called @10Hz to handle loading from eeprom
    void update() {
        _poly_loader.update();
    }

    /// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
    bool pre_arm_check(const char* &fail_msg) const;

    ///
    /// methods to check we are within the boundaries and recover
    ///

    /// check - returns the fence type that has been breached (if any)
    uint8_t check();

    // returns true if the destination is within fence (used to reject waypoints outside the fence)
    bool check_destination_within_fence(const class Location& loc);

    /// get_breaches - returns bit mask of the fence types that have been breached
    uint8_t get_breaches() const { return _breached_fences; }

    /// get_breach_time - returns time the fence was breached
    uint32_t get_breach_time() const { return _breach_time; }

    /// get_breach_count - returns number of times we have breached the fence
    uint16_t get_breach_count() const { return _breach_count; }

    /// get_breach_distance - returns maximum distance in meters outside
    /// of the given fences.  fence_type is a bitmask here.
    float get_breach_distance(uint8_t fence_type) const;

    /// get_action - getter for user requested action on limit breach
    uint8_t get_action() const { return _action.get(); }

    /// get_safe_alt - returns maximum safe altitude (i.e. alt_max - margin)
    float get_safe_alt_max() const { return _alt_max - _margin; }

    /// get_safe_alt_min - returns the minimum safe altitude (i.e. alt_min + margin)
    float get_safe_alt_min() const { return _alt_min + _margin; }

    /// get_radius - returns the fence radius in meters
    float get_radius() const { return _circle_radius.get(); }

    /// get_margin - returns the fence margin in meters
    float get_margin() const { return _margin.get(); }

    /// get_return_rally - returns whether returning to fence return point or rally point
    uint8_t get_return_rally() const { return _ret_rally; }
    
    /// get_return_rally - returns whether returning to fence return point or rally point
    float get_return_altitude() const { return _ret_altitude; }

    /// manual_recovery_start - caller indicates that pilot is re-taking manual control so fence should be disabled for 10 seconds
    ///     should be called whenever the pilot changes the flight mode
    ///     has no effect if no breaches have occurred
    void manual_recovery_start();

    // methods for mavlink SYS_STATUS message (send_sys_status)
    bool sys_status_present() const;
    bool sys_status_enabled() const;
    bool sys_status_failed() const;

    AC_PolyFence_loader &polyfence();
    const AC_PolyFence_loader &polyfence() const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AC_Fence *_singleton;

    /// check_fence_alt_max - true if max alt fence has been newly breached
    bool check_fence_alt_max();

    /// check_fence_alt_min - true if min alt fence has been newly breached
    bool check_fence_alt_min();

    /// check_fence_polygon - true if polygon fence has been newly breached
    bool check_fence_polygon();

    /// check_fence_circle - true if circle fence has been newly breached
    bool check_fence_circle();

    /// record_breach - update breach bitmask, time and count
    void record_breach(uint8_t fence_type);

    /// clear_breach - update breach bitmask, time and count
    void clear_breach(uint8_t fence_type);

    // additional checks for the different fence types:
    bool pre_arm_check_polygon(const char* &fail_msg) const;
    bool pre_arm_check_circle(const char* &fail_msg) const;
    bool pre_arm_check_alt(const char* &fail_msg) const;

    // parameters
    AP_Int8         _enabled;               // fence enable/disable control
    AP_Int8         _auto_enabled;          // top level flag for auto enabling fence
    AP_Int8         _enabled_fences;        // bit mask holding which fences are enabled
    AP_Int8         _action;                // recovery action specified by user
    AP_Float        _alt_max;               // altitude upper limit in meters
    AP_Float        _alt_min;               // altitude lower limit in meters
    AP_Float        _circle_radius;         // circle fence radius in meters
    AP_Float        _margin;                // distance in meters that autopilot's should maintain from the fence to avoid a breach
    AP_Int8         _total;                 // number of polygon points saved in eeprom
    AP_Int8         _ret_rally;             // return to fence return point or rally point/home
    AP_Int16        _ret_altitude;          // return to this altitude

    // backup fences
    float           _alt_max_backup;        // backup altitude upper limit in meters used to refire the breach if the vehicle continues to move further away
    float           _alt_min_backup;        // backup altitude lower limit in meters used to refire the breach if the vehicle continues to move further away
    float           _circle_radius_backup;  // backup circle fence radius in meters used to refire the breach if the vehicle continues to move further away

    // breach distances
    float           _alt_max_breach_distance;   // distance above the altitude max
    float           _alt_min_breach_distance;   // distance below the altitude min
    float           _circle_breach_distance;    // distance beyond the circular fence

    // other internal variables
    bool            _floor_enabled;         // fence floor is enabled
    float           _home_distance;         // distance from home in meters (provided by main code)
    float           _curr_alt;


    // breach information
    uint8_t         _breached_fences;       // bitmask holding the fence type that was breached (i.e. AC_FENCE_TYPE_ALT_MIN, AC_FENCE_TYPE_CIRCLE)
    uint32_t        _breach_time;           // time of last breach in milliseconds
    uint16_t        _breach_count;          // number of times we have breached the fence
    uint32_t _last_breach_notify_sent_ms;  // last time we sent a message about newly-breaching the fences

    uint32_t        _manual_recovery_start_ms;  // system time in milliseconds that pilot re-took manual control


    AC_PolyFence_loader _poly_loader{_total}; // polygon fence
};

namespace AP {
    AC_Fence *fence();
};

#endif // AP_FENCE_ENABLED
