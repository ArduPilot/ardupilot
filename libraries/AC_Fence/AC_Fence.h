#pragma once

#include "AC_Fence_config.h"

#if AP_FENCE_ENABLED

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/ExpandingString.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_Fence/AC_PolyFence_loader.h>

// bit masks for enabled fence types.  Used for TYPE parameter
#define AC_FENCE_TYPE_ALT_MAX                       1       // high alt fence which usually initiates an RTL
#define AC_FENCE_TYPE_CIRCLE                        2       // circular horizontal fence (usually initiates an RTL)
#define AC_FENCE_TYPE_POLYGON                       4       // polygon horizontal fence
#define AC_FENCE_TYPE_ALT_MIN                       8       // low alt fence which usually initiates an RTL
#define AC_FENCE_ARMING_FENCES  (AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)
#define AC_FENCE_ALL_FENCES (AC_FENCE_ARMING_FENCES | AC_FENCE_TYPE_ALT_MIN)

// give up distance
#define AC_FENCE_GIVE_UP_DISTANCE                   100.0f  // distance outside the fence at which we should give up and just land.  Note: this is not used by library directly but is intended to be used by the main code

class AC_Fence
{
public:
    friend class AC_PolyFence_loader;

    enum class AutoEnable : uint8_t
    {
        ALWAYS_DISABLED = 0,
        ENABLE_ON_AUTO_TAKEOFF = 1, // enable on auto takeoff
        ENABLE_DISABLE_FLOOR_ONLY = 2,  // enable on takeoff but disable floor on landing
        ONLY_WHEN_ARMED = 3 // enable on arming
    };

    enum class MavlinkFenceActions : uint16_t
    {
        DISABLE_FENCE = 0,
        ENABLE_FENCE = 1,
        DISABLE_ALT_MIN_FENCE = 2
    };

    // valid actions should a fence be breached
    enum class Action : uint8_t
    {
        REPORT_ONLY           = 0,       // report to GCS that boundary has been breached but take no further action
        RTL_AND_LAND          = 1,       // return to launch and, if that fails, land
        ALWAYS_LAND           = 2,       // always land
        SMART_RTL             = 3,       // smartRTL, if that fails, RTL, it that still fails, land
        BRAKE                 = 4,       // brake, if that fails, land
        SMART_RTL_OR_LAND     = 5,       // SmartRTL, if that fails, Land
        GUIDED                = 6,       // guided mode, with target waypoint as fence return point
        GUIDED_THROTTLE_PASS  = 7,       // guided mode, but pilot retains manual throttle control
    };

    AC_Fence();

    /* Do not allow copies */
    CLASS_NO_COPY(AC_Fence);

    void init() {
        _poly_loader.init();
    }

    // get singleton instance
    static AC_Fence *get_singleton() { return _singleton; }

    /// enable - allows fence to be enabled/disabled.
    /// returns a bitmask of fences that were changed
    uint8_t enable(bool value, uint8_t fence_types, bool update_auto_mask = true);

    /// enable_configured - allows configured fences to be enabled/disabled.
    /// returns a bitmask of fences that were changed
    uint8_t enable_configured(bool value) { return enable(value, _configured_fences, true); }

    /// auto_enabled - automaticaly enable/disable fence depending of flight status
    AutoEnable auto_enabled() const { return static_cast<AutoEnable>(_auto_enabled.get()); }

    /// enable_floor - allows fence floor to be enabled/disabled. Note this does not update the eeprom saved value
    void enable_floor();

    /// disable_floor - allows fence floor to be enabled/disabled. Note this does not update the eeprom saved value
    void disable_floor();

    /// auto_enable_fence_on_takeoff - auto enables the fence. Called after takeoff conditions met
    void auto_enable_fence_after_takeoff();

    /// auto_enable_fences_on_arming - auto enables all applicable fences on arming
    void auto_enable_fence_on_arming();

    /// auto_disable_fences_on_disarming - auto disables all applicable fences on disarming
    void auto_disable_fence_on_disarming();

    uint8_t get_auto_disable_fences(void) const;

    /// auto_enable_fence_floor - auto enables fence floor once desired altitude has been reached.
    bool auto_enable_fence_floor();

    /// enabled - returns whether fencing is enabled or not
    bool enabled() const { return _enabled_fences; }

    /// present - returns true if any of the provided types is present
    uint8_t present() const;

    /// get_enabled_fences - returns bitmask of enabled fences
    uint8_t get_enabled_fences() const;

    // should be called @10Hz to handle loading from eeprom
    void update();

    /// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
    bool pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const;

    ///
    /// methods to check we are within the boundaries and recover
    ///

    /// check - returns the fence type that has been breached (if any)
    /// disabled_fences can be used to disable fences for certain conditions (e.g. landing)
    uint8_t check(bool disable_auto_fence = false);

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

    /// get a user-friendly list of fences
    static void get_fence_names(uint8_t fences, ExpandingString& msg);

    // print a message about the fences to the GCS
    void print_fence_message(const char* msg, uint8_t fences) const;

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

    enum class OPTIONS {
        DISABLE_MODE_CHANGE = 1U << 0,
        INCLUSION_UNION = 1U << 1,
    };
    static bool option_enabled(OPTIONS opt, const AP_Int16 &options) {
        return (options.get() & int16_t(opt)) != 0;
    }
    bool option_enabled(OPTIONS opt) const {
        return option_enabled(opt, _options);
    }

    static const struct AP_Param::GroupInfo var_info[];

#if AP_SDCARD_STORAGE_ENABLED
    bool failed_sdcard_storage(void) const {
        return _poly_loader.failed_sdcard_storage();
    }
#endif

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
    bool pre_arm_check_polygon(char *failure_msg, const uint8_t failure_msg_len) const;
    bool pre_arm_check_circle(char *failure_msg, const uint8_t failure_msg_len) const;
    bool pre_arm_check_alt(char *failure_msg, const uint8_t failure_msg_len) const;
    // fence floor is enabled
    bool floor_enabled() const { return _enabled_fences & AC_FENCE_TYPE_ALT_MIN; }

    // parameters
    uint8_t         _enabled_fences;        // fences that are currently enabled/disabled
    bool            _last_enabled;          // value of enabled last time we checked
    AP_Int8         _enabled;               // overall feature control
    AP_Int8         _auto_enabled;          // top level flag for auto enabling fence
    uint8_t         _last_auto_enabled;     // value of auto_enabled last time we checked
    AP_Int8         _configured_fences;     // bit mask holding which fences are enabled
    AP_Int8         _action;                // recovery action specified by user
    AP_Float        _alt_max;               // altitude upper limit in meters
    AP_Float        _alt_min;               // altitude lower limit in meters
    AP_Float        _circle_radius;         // circle fence radius in meters
    AP_Float        _margin;                // distance in meters that autopilot's should maintain from the fence to avoid a breach
    AP_Int8         _total;                 // number of polygon points saved in eeprom
    AP_Int8         _ret_rally;             // return to fence return point or rally point/home
    AP_Int16        _ret_altitude;          // return to this altitude
    AP_Int16        _options;               // options bitmask, see OPTIONS enum

    // backup fences
    float           _alt_max_backup;        // backup altitude upper limit in meters used to refire the breach if the vehicle continues to move further away
    float           _alt_min_backup;        // backup altitude lower limit in meters used to refire the breach if the vehicle continues to move further away
    float           _circle_radius_backup;  // backup circle fence radius in meters used to refire the breach if the vehicle continues to move further away

    // breach distances
    float           _alt_max_breach_distance;   // distance above the altitude max
    float           _alt_min_breach_distance;   // distance below the altitude min
    float           _circle_breach_distance;    // distance beyond the circular fence

    // other internal variables
    uint8_t         _auto_enable_mask = AC_FENCE_ALL_FENCES;  // fences that can be auto-enabled or auto-disabled
    float           _home_distance;         // distance from home in meters (provided by main code)
    float           _curr_alt;


    // breach information
    uint8_t         _breached_fences;       // bitmask holding the fence type that was breached (i.e. AC_FENCE_TYPE_ALT_MIN, AC_FENCE_TYPE_CIRCLE)
    uint32_t        _breach_time;           // time of last breach in milliseconds
    uint16_t        _breach_count;          // number of times we have breached the fence
    uint32_t _last_breach_notify_sent_ms;  // last time we sent a message about newly-breaching the fences

    uint32_t        _manual_recovery_start_ms;  // system time in milliseconds that pilot re-took manual control


    AC_PolyFence_loader _poly_loader{_total, _options}; // polygon fence
};

namespace AP {
    AC_Fence *fence();
};

#endif // AP_FENCE_ENABLED
