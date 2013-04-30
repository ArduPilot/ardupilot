/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_Fence.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_Fence::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: Fence enable/disable
    // @Description: Allows you to enable (1) or disable (0) the fence functionality
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",      0,  AC_Fence,   _enabled,   0),

    // @Param: TYPE
    // @DisplayName: Fence Type
    // @Description: Enabled fence types held as bitmask
    // @Values: 0:None,1:MaxAltitude,2:Circle
    // @User: Standard
    AP_GROUPINFO("TYPE",        1,  AC_Fence,   _enabled_fences,  AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_BIG_CIRCLE),

    // @Param: ACTION
    // @DisplayName: Action to perform when the limit is breached
    // @Description: What to do on fence breach
    // @Values: 0:Report Only,1:Bounce,3:Return-to-Launch,4:Move to location
    // @User: Standard
    AP_GROUPINFO("ACTION",      2,  AC_Fence,   _action,        AC_FENCE_ACTION_RTL_AND_LAND),

    // @Param: ALT_MAX
    // @DisplayName: Fence Maximum Altitude
    // @Description: Maximum altitude allowed before geofence triggers
    // @Units: centimeters
    // @Range: 1000 100000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("ALT_MAX",     3,  AC_Fence,   _alt_max_cm,   AC_FENCE_ALT_MAX_DEFAULT),

    // @Param: RADIUS
    // @DisplayName: Circular Fence Radius
    // @Description: circle fence radius in cm
    // @Units: centimeters
    // @Range: 0 65536
    // @User: Standard
    AP_GROUPINFO("RADIUS",      4,  AC_Fence,   _radius_cm,     AC_FENCE_RADIUS_DEFAULT),
    
    AP_GROUPEND
};

/// Default constructor.
AC_Fence::AC_Fence(AP_InertialNav* inav, GPS** gps_ptr) :
    _inav(inav),
    _gps_ptr(gps_ptr)
{
    AP_Param::setup_object_defaults(this, var_info);

    // check for silly fence values
    if( _alt_max_cm < 0 ) {
        _alt_max_cm.set_and_save(AC_FENCE_ALT_MAX_DEFAULT);
    }
    if( _radius_cm < 0 ) {
        _radius_cm.set_and_save(AC_FENCE_RADIUS_DEFAULT);
    }
}

/// get_enabled_fences - returns bitmask of enabled fences
uint8_t AC_Fence::get_enabled_fences()
{
    if(!_enabled) {
        return false;
    }else{
        return _enabled_fences;
    }
}

/// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
bool AC_Fence::pre_arm_check() const
{
    // if not enabled or not fence set-up always return true
    if(!_enabled || _enabled_fences == AC_FENCE_TYPE_NONE) {
        return true;
    }

    // check no limits are currently breached
    if(_breached_fences != AC_FENCE_TYPE_NONE) {
        return false;
    }

    // if we have horizontal limits enabled, check inertial nav position is ok
    if((_enabled_fences & (AC_FENCE_TYPE_CIRCLE|AC_FENCE_TYPE_BIG_CIRCLE))>0 && !_inav->position_ok()) {
        return false;
    }

    // if we got this far everything must be ok
    return true;
}

/// check_fence - returns the fence type that has been breached (if any)
uint8_t AC_Fence::check_fence()
{
    uint8_t ret = AC_FENCE_TYPE_NONE;

    // return immediately if disabled
    if(!_enabled || _enabled_fences == AC_FENCE_TYPE_NONE) {
        return AC_FENCE_TYPE_NONE;
    }

    // get current position
    Vector3f curr = _inav->get_position();

    // check max altitude
    if( (_enabled_fences & AC_FENCE_TYPE_ALT_MAX) > 0 ) {
        if(curr.z >= _alt_max_cm ) {
            // ensure it's a new breach
            if((_breached_fences & AC_FENCE_TYPE_ALT_MAX) == 0) {
                // record that we have breached the upper limit
                record_breach(AC_FENCE_TYPE_ALT_MAX);
                ret = ret | AC_FENCE_TYPE_ALT_MAX;
            }
        }else{
            clear_breach(AC_FENCE_TYPE_ALT_MAX);
        }
    }

    // circle fence check
    if( (_enabled_fences & AC_FENCE_TYPE_CIRCLE) > 0 ) {
        if( _home_distance_cm >= _radius_cm ) {
            // ensure it's a new breach
            if((_breached_fences & AC_FENCE_TYPE_CIRCLE) == 0) {
                // record that we have breached the circular distance limit
                record_breach(AC_FENCE_TYPE_CIRCLE);
                ret = ret | AC_FENCE_TYPE_CIRCLE;
            }
        }else{
            clear_breach(AC_FENCE_TYPE_CIRCLE);
        }
    }

    // big circle fence check
    if( (_enabled_fences & AC_FENCE_TYPE_BIG_CIRCLE) > 0 ) {
        if( _home_distance_cm >= _radius_cm * 2 ) {
            // ensure it's a new breach
            if((_breached_fences & AC_FENCE_TYPE_BIG_CIRCLE) == 0) {
                // record that we have breached the circular distance limit
                record_breach(AC_FENCE_TYPE_BIG_CIRCLE);
                ret = ret | AC_FENCE_TYPE_BIG_CIRCLE;
            }
        }else{
            clear_breach(AC_FENCE_TYPE_BIG_CIRCLE);
        }
    }

    // return any new breaches that have occurred
    return ret;

    // To-Do: add min alt and polygon check
    //outside = Polygon_outside(location, &geofence_state->boundary[1], geofence_state->num_points-1);
}

/// record_breach - update breach bitmask, time and count
void AC_Fence::record_breach(uint8_t fence_type)
{
    // if we haven't already breached a limit, update the breach time
    if( _breached_fences == AC_FENCE_TYPE_NONE ) {
        _breach_time = hal.scheduler->millis();
    }

    // update breach count
    if( _breach_count < 65500) {
        _breach_count++;
    }

    // update bitmask
    _breached_fences = _breached_fences | fence_type;
}

/// clear_breach - update breach bitmask, time and count
void AC_Fence::clear_breach(uint8_t fence_type)
{
    // return immediately if this fence type was not breached
    if( (_breached_fences & fence_type) == 0 ) {
        return;
    }

    // update bitmask
    _breached_fences = _breached_fences & ~fence_type;

    // if all breaches cleared, clear the breach time
    if( _breached_fences == AC_FENCE_TYPE_NONE ) {
        _breach_time = 0;
    }
}