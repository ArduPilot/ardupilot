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
    // @Values: 0:None,1:Altitude,2:Circle,3:Altitude and Circle
    // @User: Standard
    AP_GROUPINFO("TYPE",        1,  AC_Fence,   _enabled_fences,  AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE),

    // @Param: ACTION
    // @DisplayName: Fence Action
    // @Description: What action should be taken when fence is breached
    // @Values: 0:Report Only,1:RTL or Land
    // @User: Standard
    AP_GROUPINFO("ACTION",      2,  AC_Fence,   _action,        AC_FENCE_ACTION_RTL_AND_LAND),

    // @Param: ALT_MAX
    // @DisplayName: Fence Maximum Altitude
    // @Description: Maximum altitude allowed before geofence triggers
    // @Units: Meters
    // @Range: 10 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MAX",     3,  AC_Fence,   _alt_max,       AC_FENCE_ALT_MAX_DEFAULT),

    // @Param: RADIUS
    // @DisplayName: Circular Fence Radius
    // @Description: Circle fence radius which when breached will cause an RTL
    // @Units: Meters
    // @Range: 30 10000
    // @User: Standard
    AP_GROUPINFO("RADIUS",      4,  AC_Fence,   _circle_radius, AC_FENCE_CIRCLE_RADIUS_DEFAULT),

    // @Param: MARGIN
    // @DisplayName: Fence Margin
    // @Description: Distance that autopilot's should maintain from the fence to avoid a breach
    // @Units: Meters
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("MARGIN",      5,  AC_Fence,   _margin, AC_FENCE_MARGIN_DEFAULT),
    
    AP_GROUPEND
};

/// Default constructor.
AC_Fence::AC_Fence(const AP_InertialNav* inav) :
    _inav(inav),
    _alt_max_backup(0),
    _circle_radius_backup(0),
    _alt_max_breach_distance(0),
    _circle_breach_distance(0),
    _home_distance(0),
    _breached_fences(AC_FENCE_TYPE_NONE),
    _breach_time(0),
    _breach_count(0),
    _manual_recovery_start_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // check for silly fence values
    if (_alt_max < 0.0f) {
        _alt_max.set_and_save(AC_FENCE_ALT_MAX_DEFAULT);
    }
    if (_circle_radius < 0) {
        _circle_radius.set_and_save(AC_FENCE_CIRCLE_RADIUS_DEFAULT);
    }
}

/// get_enabled_fences - returns bitmask of enabled fences
uint8_t AC_Fence::get_enabled_fences() const
{
    if (!_enabled) {
        return AC_FENCE_TYPE_NONE;
    }else{
        return _enabled_fences;
    }
}

/// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
bool AC_Fence::pre_arm_check() const
{
    // if not enabled or not fence set-up always return true
    if (!_enabled || _enabled_fences == AC_FENCE_TYPE_NONE) {
        return true;
    }

    // check no limits are currently breached
    if (_breached_fences != AC_FENCE_TYPE_NONE) {
        return false;
    }

    // if we have horizontal limits enabled, check inertial nav position is ok
    if ((_enabled_fences & AC_FENCE_TYPE_CIRCLE)!=0 && !_inav->position_ok()) {
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
    if (!_enabled || _enabled_fences == AC_FENCE_TYPE_NONE) {
        return AC_FENCE_TYPE_NONE;
    }

    // check if pilot is attempting to recover manually
    if (_manual_recovery_start_ms != 0) {
        // we ignore any fence breaches during the manual recovery period which is about 10 seconds
        if ((hal.scheduler->millis() - _manual_recovery_start_ms) < AC_FENCE_MANUAL_RECOVERY_TIME_MIN) {
            return AC_FENCE_TYPE_NONE;
        } else {
            // recovery period has passed so reset manual recovery time and continue with fence breach checks
            _manual_recovery_start_ms = 0;
        }
    }

    // get current altitude in meters
    float curr_alt = _inav->get_altitude() * 0.01f;

    // altitude fence check
    if ((_enabled_fences & AC_FENCE_TYPE_ALT_MAX) != 0) {

        // check if we are over the altitude fence
        if( curr_alt >= _alt_max ) {

            // record distance above breach
            _alt_max_breach_distance = curr_alt - _alt_max;

            // check for a new breach or a breach of the backup fence
            if ((_breached_fences & AC_FENCE_TYPE_ALT_MAX) == 0 || (_alt_max_backup != 0.0f && curr_alt >= _alt_max_backup)) {

                // record that we have breached the upper limit
                record_breach(AC_FENCE_TYPE_ALT_MAX);
                ret = ret | AC_FENCE_TYPE_ALT_MAX;

                // create a backup fence 20m higher up
                _alt_max_backup = curr_alt + AC_FENCE_ALT_MAX_BACKUP_DISTANCE;
            }
        }else{
            // clear alt breach if present
            if ((_breached_fences & AC_FENCE_TYPE_ALT_MAX) != 0) {
                clear_breach(AC_FENCE_TYPE_ALT_MAX);
                _alt_max_backup = 0.0f;
                _alt_max_breach_distance = 0.0f;
            }
        }
    }

    // circle fence check
    if ((_enabled_fences & AC_FENCE_TYPE_CIRCLE) != 0 ) {

        // check if we are outside the fence
        if (_home_distance >= _circle_radius) {

            // record distance outside the fence
            _circle_breach_distance = _home_distance - _circle_radius;

            // check for a new breach or a breach of the backup fence
            if ((_breached_fences & AC_FENCE_TYPE_CIRCLE) == 0 || (_circle_radius_backup != 0.0f && _home_distance >= _circle_radius_backup)) {

                // record that we have breached the circular distance limit
                record_breach(AC_FENCE_TYPE_CIRCLE);
                ret = ret | AC_FENCE_TYPE_CIRCLE;

                // create a backup fence 20m further out
                _circle_radius_backup = _home_distance + AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE;
            }
        }else{
            // clear circle breach if present
            if ((_breached_fences & AC_FENCE_TYPE_CIRCLE) != 0) {
                clear_breach(AC_FENCE_TYPE_CIRCLE);
                _circle_radius_backup = 0.0f;
                _circle_breach_distance = 0.0f;
            }
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
    if (_breached_fences == AC_FENCE_TYPE_NONE) {
        _breach_time = hal.scheduler->millis();
    }

    // update breach count
    if (_breach_count < 65500) {
        _breach_count++;
    }

    // update bitmask
    _breached_fences |= fence_type;
}

/// clear_breach - update breach bitmask, time and count
void AC_Fence::clear_breach(uint8_t fence_type)
{
    // return immediately if this fence type was not breached
    if ((_breached_fences & fence_type) == 0) {
        return;
    }

    // update bitmask
    _breached_fences &= ~fence_type;
}

/// get_breach_distance - returns distance in meters outside of the given fence
float AC_Fence::get_breach_distance(uint8_t fence_type) const
{
    switch (fence_type) {
        case AC_FENCE_TYPE_ALT_MAX:
            return _alt_max_breach_distance;
            break;
        case AC_FENCE_TYPE_CIRCLE:
            return _circle_breach_distance;
            break;
        case AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE:
            return max(_alt_max_breach_distance,_circle_breach_distance);
    }

    // we don't recognise the fence type so just return 0
    return 0;
}

/// manual_recovery_start - caller indicates that pilot is re-taking manual control so fence should be disabled for 10 seconds
///     has no effect if no breaches have occurred
void AC_Fence::manual_recovery_start()
{
    // return immediate if we haven't breached a fence
    if (_breached_fences == AC_FENCE_TYPE_NONE) {
        return;
    }

    // record time pilot began manual recovery
    _manual_recovery_start_ms = hal.scheduler->millis();
}
