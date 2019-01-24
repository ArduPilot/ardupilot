#pragma once

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Fence/AC_PolyFence_loader.h>
#include <AP_Common/Location.h>

// bit masks for enabled fence types.  Used for TYPE parameter
#define AC_FENCE_TYPE_ALT_MAX                       1       // high alt fence which usually initiates an RTL
#define AC_FENCE_TYPE_CIRCLE                        2       // circular horizontal fence (usually initiates an RTL)
#define AC_FENCE_TYPE_POLYGON                       4       // polygon horizontal fence

// valid actions should a fence be breached
#define AC_FENCE_ACTION_REPORT_ONLY                 0       // report to GCS that boundary has been breached but take no further action
#define AC_FENCE_ACTION_RTL_AND_LAND                1       // return to launch and, if that fails, land
#define AC_FENCE_ACTION_ALWAYS_LAND                 2       // always land
#define AC_FENCE_ACTION_SMART_RTL                   3       // smartRTL, if that fails, RTL, it that still fails, land
#define AC_FENCE_ACTION_BRAKE                       4       // brake, if that fails, land

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
    AC_Fence(const AP_AHRS_NavEKF &ahrs);

    /* Do not allow copies */
    AC_Fence(const AC_Fence &other) = delete;
    AC_Fence &operator=(const AC_Fence&) = delete;

    /// enable - allows fence to be enabled/disabled.  Note: this does not update the eeprom saved value
    void enable(bool value);

    /// enabled - returns true if fence is enabled
    bool enabled() const { return _enabled; }

    /// get_enabled_fences - returns bitmask of enabled fences
    uint8_t get_enabled_fences() const;

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

    ///
    /// polygon related methods
    ///

    /// returns pointer to array of polygon points and num_points is filled in with the total number
    Vector2f* get_polygon_points(uint16_t& num_points) const;

    /// returns true if we've breached the polygon boundary.  simple passthrough to underlying _poly_loader object
    bool boundary_breached(const Vector2f& location, uint16_t num_points, const Vector2f* points) const;

    /// handler for polygon fence messages with GCS
    void handle_msg(GCS_MAVLINK &link, mavlink_message_t* msg);

    static const struct AP_Param::GroupInfo var_info[];

    // methods for mavlink SYS_STATUS message (send_sys_status)
    bool sys_status_present() const;
    bool sys_status_enabled() const;
    bool sys_status_failed() const;

private:

    /// check_fence_alt_max - true if alt fence has been newly breached
    bool check_fence_alt_max();

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

    /// load polygon points stored in eeprom into boundary array and perform validation.  returns true if load successfully completed
    bool load_polygon_from_eeprom(bool force_reload = false);

    // pointers to other objects we depend upon
    const AP_AHRS_NavEKF& _ahrs;

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
    uint8_t         _breached_fences;       // bitmask holding the fence type that was breached (i.e. AC_FENCE_TYPE_ALT_MIN, AC_FENCE_TYPE_CIRCLE)
    uint32_t        _breach_time;           // time of last breach in milliseconds
    uint16_t        _breach_count;          // number of times we have breached the fence

    uint32_t        _manual_recovery_start_ms;  // system time in milliseconds that pilot re-took manual control

    // polygon fence variables
    AC_PolyFence_loader _poly_loader;               // helper for loading/saving polygon points
    Vector2f        *_boundary = nullptr;           // array of boundary points.  Note: point 0 is the return point
    uint8_t         _boundary_num_points = 0;       // number of points in the boundary array (should equal _total parameter after load has completed)
    bool            _boundary_create_attempted = false; // true if we have attempted to create the boundary array
    bool            _boundary_loaded = false;       // true if boundary array has been loaded from eeprom
    bool            _boundary_valid = false;        // true if boundary forms a closed polygon
};
