#pragma once

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AC_Fence/AC_PolyFence_loader.h>
#include <AP_Common/Location.h>

// default boundaries
#define AC_FENCE_ALT_MAX_DEFAULT                    100.0f  // default max altitude is 100m
#define AC_FENCE_ALT_MIN_DEFAULT                    -10.0f  // default maximum depth in meters
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
    AC_Fence();

    enum class Type : uint8_t {
        // bit masks for enabled fence types.  Used for TYPE parameter
        ALT_MAX  = 1, // high alt fence which usually initiates an RTL
        CIRCLE   = 2, // circular horizontal fence (usually initiates an RTL)
        POLYGON  = 4, // polygon horizontal fence
    };
    typedef uint8_t TypeMask;
        // Type operator |(Type lhs, Type rhs)
        // {
        //     return static_cast<Type> (
        //         static_cast<uint8_t>(lhs) |
        //         static_cast<uint8_t>(rhs)
        //         );
        // }

    enum class Action : uint8_t {
        // valid actions should a fence be breached
        REPORT_ONLY  = 0, // report to GCS that boundary has been breached but take no further action
        RTL_AND_LAND = 1, // return to launch and, if that fails, land
        ALWAYS_LAND  = 2, // always land
        SMART_RTL    = 3, // smartRTL, if that fails, RTL, it that still fails, land
        BRAKE        = 4, // brake, if that fails, land
    };

    /* Do not allow copies */
    AC_Fence(const AC_Fence &other) = delete;
    AC_Fence &operator=(const AC_Fence&) = delete;

    void init() {
        _poly_loader.init();
    }

    // get singleton instance
    static AC_Fence *get_singleton() { return _singleton; }

    /// enable - allows fence to be enabled/disabled.  Note: this does not update the eeprom saved value
    void enable(bool value);

    /// enabled - returns true if fence is enabled
    bool enabled() const { return _enabled; }

    /// get_enabled_fences - returns bitmask of enabled fences
    TypeMask get_enabled_fences() const;

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
    bool check_destination_within_fence(const Location& loc);

    /// get_breaches - returns bit mask of the fence types that have been breached
    TypeMask get_breaches() const { return _breached_fences; }

    /// get_breach_time - returns time the fence was breached
    uint32_t get_breach_time() const { return _breach_time; }

    /// get_breach_count - returns number of times we have breached the fence
    uint16_t get_breach_count() const { return _breach_count; }

    /// get_breach_distance - returns maximum distance in meters outside
    /// of the given fences.  fence_type is a bitmask here.
    float get_breach_distance(uint8_t fence_type) const;

    /// get_action - getter for user requested action on limit breach
    Action get_action() const { return (Action)_action.get(); }

    /// get_safe_alt - returns maximum safe altitude (i.e. alt_max - margin)
    float get_safe_alt_max() const { return _alt_max - _margin; }

    /// get_safe_alt_min - returns the minimum safe altitude (i.e. alt_min - margin)
    float get_safe_alt_min() const { return _alt_min + _margin; }

    /// get_radius - returns the fence radius in meters
    float get_radius() const { return _circle_radius.get(); }

    /// get_margin - returns the fence margin in meters
    float get_margin() const { return _margin.get(); }

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

    /// check_fence_alt_max - true if alt fence has been newly breached
    bool check_fence_alt_max();

    /// check_fence_polygon - true if polygon fence has been newly breached
    bool check_fence_polygon();

    /// check_fence_circle - true if circle fence has been newly breached
    bool check_fence_circle();

    /// record_breach - update breach bitmask, time and count
    void record_breach(Type fence_type);

    /// clear_breach - update breach bitmask, time and count
    void clear_breach(Type fence_type) { clear_breach((TypeMask)fence_type); }
    void clear_breach(TypeMask fence_type);

    // additional checks for the different fence types:
    bool pre_arm_check_polygon(const char* &fail_msg) const;
    bool pre_arm_check_circle(const char* &fail_msg) const;
    bool pre_arm_check_alt(const char* &fail_msg) const;

    // parameters
    AP_Int8         _enabled;               // top level enable/disable control
    AP_Int8         _enabled_fences;        // bit mask holding which fences are enabled
    AP_Int8         _action;                // recovery action specified by user
    AP_Float        _alt_max;               // altitude upper limit in meters
    AP_Float        _alt_min;               // altitude lower limit in meters
    AP_Float        _circle_radius;         // circle fence radius in meters
    AP_Float        _margin;                // distance in meters that autopilot's should maintain from the fence to avoid a breach
    AP_Int8         _total;                 // number of polygon points saved in eeprom

    // backup fences
    float           _alt_max_backup;        // backup altitude upper limit in meters used to refire the breach if the vehicle continues to move further away
    float           _circle_radius_backup;  // backup circle fence radius in meters used to refire the breach if the vehicle continues to move further away

    // breach distances
    float           _alt_max_breach_distance;   // distance above the altitude max
    float           _circle_breach_distance;    // distance beyond the circular fence

    // other internal variables
    float           _home_distance;         // distance from home in meters (provided by main code)
    float _curr_alt;


    // breach information
    TypeMask         _breached_fences;       // bitmask holding the fence type that was breached (i.e. AC_FENCE_TYPE_ALT_MIN, AC_FENCE_TYPE_CIRCLE)
    uint32_t        _breach_time;           // time of last breach in milliseconds
    uint16_t        _breach_count;          // number of times we have breached the fence

    uint32_t        _manual_recovery_start_ms;  // system time in milliseconds that pilot re-took manual control

    AC_PolyFence_loader _poly_loader{_total}; // polygon fence
};

inline std::underlying_type<AC_Fence::Type>::type operator & (AC_Fence::TypeMask lhs, AC_Fence::Type rhs)
{
    return static_cast<std::underlying_type<AC_Fence::Type>::type> (
        lhs &
        static_cast<std::underlying_type<AC_Fence::Type>::type>(rhs)
        );
}

inline AC_Fence::TypeMask operator | (AC_Fence::Type lhs, AC_Fence::Type rhs)
{
    return static_cast<AC_Fence::TypeMask> (
        static_cast<std::underlying_type<AC_Fence::Type>::type>(lhs) |
        static_cast<std::underlying_type<AC_Fence::Type>::type>(rhs)
        );
}

inline AC_Fence::TypeMask operator | (AC_Fence::TypeMask lhs, AC_Fence::Type rhs)
{
    return static_cast<AC_Fence::TypeMask> (
        lhs |
        static_cast<std::underlying_type<AC_Fence::Type>::type>(rhs)
        );
}

inline AC_Fence::TypeMask operator |= (AC_Fence::TypeMask &lhs, AC_Fence::Type rhs)
{
    lhs |= static_cast<std::underlying_type<AC_Fence::Type>::type>(rhs);
    return lhs;
}

namespace AP {
    AC_Fence *fence();
};
