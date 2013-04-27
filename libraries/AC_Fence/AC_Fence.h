/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_FENCE_H
#define AC_FENCE_H

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_InertialNav.h>     // Inertial Navigation library

// bit masks for enabled fence types.  Used for TYPE parameter
#define AC_FENCE_TYPE_NONE                  0       // fence disabled
#define AC_FENCE_TYPE_ALT_MAX               1       // max alt fence enabled
#define AC_FENCE_TYPE_CIRCLE                2       // circular horizontal fence
#define AC_FENCE_TYPE_BIG_CIRCLE            4       // circular horizonal fence double the normal radius used to force land when all hope of RTL is lost

// valid actions should a fence be breached
#define AC_FENCE_ACTION_REPORT_ONLY         0       // report to GCS that boundary has been breached but take no further action
#define AC_FENCE_ACTION_RTL_AND_LAND        1       // return to launch and, if that fails, land

// default boundaries
#define AC_FENCE_ALT_MAX_DEFAULT            15000   // default max altitude is 150m
#define AC_FENCE_RADIUS_DEFAULT             30000   // default circular fence radius is 300m
#define AC_FENCE_SAFETY_MARGIN_DEFAULT      200.0f  // default distance within the fence to move to after bouncing off a wall

class AC_Fence
{
public:

    /// Constructor
    AC_Fence(AP_InertialNav* inav, GPS** gps_ptr);

    /// enable - allows fence to be enabled/disabled.  Note: this does not update the eeprom saved value
    void enable(bool true_false) { _enabled = true_false; }

    /// enabled - returns true if fence is enabled
    bool enabled() { return _enabled; }

    /// get_enabled_fences - returns bitmask of enabled fences
    uint8_t get_enabled_fences();

    /// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
    bool pre_arm_check() const;

    ///
    /// methods to check we are within the boundaries and recover
    ///

    /// check_fence - returns the fence type that has been breached (if any)
    uint8_t check_fence();

    /// get_breaches - returns bit mask of the fence types that have been breached
    uint8_t get_breaches() const { return _breached_fences; }

    /// get_breach_time - returns time the fence was breached
    uint32_t get_breach_time() const { return _breach_time; }

    /// get_breach_count - returns number of times we have breached the fence
    uint32_t get_breach_count() const { return _breach_count; }

    /// get_action - getter for user requested action on limit breach
    uint8_t get_action() const { return _action.get(); }
    
    ///
    /// time saving methods to piggy-back on main code's calculations
    ///

    /// set_home_distance - update home distance - required for circular horizontal fence monitoring
    void set_home_distance(float distance_cm) { _home_distance_cm = distance_cm; }

    static const struct AP_Param::GroupInfo var_info[];

private:

    /// record_breach - update breach bitmask, time and count
    void record_breach(uint8_t fence_type);

    /// clear_breach - update breach bitmask, time and count
    void clear_breach(uint8_t fence_type);

    // pointers to other objects we depend upon
    AP_InertialNav* _inav;
    GPS**           _gps_ptr;              // pointer to pointer to gps

    // parameters
    AP_Int8         _enabled;               // top level enable/disable control
    AP_Int8         _enabled_fences;        // bit mask holding which fences are enabled
    AP_Int8         _action;                // recovery action specified by user
    AP_Float        _alt_max_cm;            // altitude upper limit in cm
    AP_Float        _radius_cm;             // circle fence radius in cm

    // other internal variables
    float           _home_distance_cm;      // distance from home in cm (provided by main code)

    // breach information
    uint8_t         _breached_fences;       // bitmask holding the fence type that was breached (i.e. AC_FENCE_TYPE_ALT_MIN, AC_FENCE_TYPE_CIRCLE)
    uint32_t        _breach_time;           // time of last breach in milliseconds
    uint16_t        _breach_count;          // number of times we have breached the fence
};
#endif	// AC_FENCE_H
