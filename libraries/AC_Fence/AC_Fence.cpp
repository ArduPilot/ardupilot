#include "AC_Fence.h"

#if AP_FENCE_ENABLED

#include <AP_Vehicle/AP_Vehicle_Type.h>

#ifndef AC_FENCE_DUMMY_METHODS_ENABLED
#define AC_FENCE_DUMMY_METHODS_ENABLED  (!(APM_BUILD_TYPE(APM_BUILD_Rover) | APM_BUILD_COPTER_OR_HELI | APM_BUILD_TYPE(APM_BUILD_ArduPlane) | APM_BUILD_TYPE(APM_BUILD_ArduSub) | (AP_FENCE_ENABLED == 1)))
#endif

#if !AC_FENCE_DUMMY_METHODS_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_Rover)
#define AC_FENCE_TYPE_DEFAULT AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AC_FENCE_TYPE_DEFAULT AC_FENCE_TYPE_POLYGON
#else
#define AC_FENCE_TYPE_DEFAULT AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON
#endif

const AP_Param::GroupInfo AC_Fence::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Fence enable/disable
    // @Description: Allows you to enable (1) or disable (0) the fence functionality
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",      0,  AC_Fence,   _enabled,   0),

    // @Param: TYPE
    // @DisplayName: Fence Type
    // @Description: Enabled fence types held as bitmask
    // @Bitmask{Rover}: 1:Circle,2:Polygon
    // @Bitmask{Copter, Plane, Sub}: 0:Max altitude,1:Circle,2:Polygon,3:Min altitude
    // @User: Standard
    AP_GROUPINFO("TYPE",        1,  AC_Fence,   _enabled_fences,  AC_FENCE_TYPE_DEFAULT),

    // @Param: ACTION
    // @DisplayName: Fence Action
    // @Description: What action should be taken when fence is breached
    // @Values{Copter}: 0:Report Only,1:RTL or Land,2:Always Land,3:SmartRTL or RTL or Land,4:Brake or Land,5:SmartRTL or Land
    // @Values{Rover}: 0:Report Only,1:RTL or Hold,2:Hold,3:SmartRTL or RTL or Hold,4:SmartRTL or Hold
    // @Values{Plane}: 0:Report Only,1:RTL,6:Guided,7:GuidedThrottlePass
    // @Values: 0:Report Only,1:RTL or Land
    // @User: Standard
    AP_GROUPINFO("ACTION",      2,  AC_Fence,   _action,        AC_FENCE_ACTION_RTL_AND_LAND),

    // @Param{Copter, Plane, Sub}: ALT_MAX
    // @DisplayName: Fence Maximum Altitude
    // @Description: Maximum altitude allowed before geofence triggers
    // @Units: m
    // @Range: 10 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FRAME("ALT_MAX", 3, AC_Fence, _alt_max, AC_FENCE_ALT_MAX_DEFAULT, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_SUB | AP_PARAM_FRAME_TRICOPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_PLANE),

    // @Param: RADIUS
    // @DisplayName: Circular Fence Radius
    // @Description: Circle fence radius which when breached will cause an RTL
    // @Units: m
    // @Range: 30 10000
    // @User: Standard
    AP_GROUPINFO("RADIUS",      4,  AC_Fence,   _circle_radius, AC_FENCE_CIRCLE_RADIUS_DEFAULT),

    // @Param: MARGIN
    // @DisplayName: Fence Margin
    // @Description: Distance that autopilot's should maintain from the fence to avoid a breach
    // @Units: m
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("MARGIN",      5,  AC_Fence,   _margin, AC_FENCE_MARGIN_DEFAULT),

    // @Param: TOTAL
    // @DisplayName: Fence polygon point total
    // @Description: Number of polygon points saved in eeprom (do not update manually)
    // @Range: 1 20
    // @User: Standard
    AP_GROUPINFO("TOTAL",       6,  AC_Fence,   _total, 0),

    // @Param{Copter, Plane, Sub}: ALT_MIN
    // @DisplayName: Fence Minimum Altitude
    // @Description: Minimum altitude allowed before geofence triggers
    // @Units: m
    // @Range: -100 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FRAME("ALT_MIN",     7,  AC_Fence,   _alt_min,       AC_FENCE_ALT_MIN_DEFAULT, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_SUB | AP_PARAM_FRAME_TRICOPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_PLANE),

    // @Param{Plane}: RET_RALLY
    // @DisplayName: Fence Return to Rally
    // @Description: Should the vehicle return to fence return point or rally point
    // @Values: 0:Fence Return Point,1:Nearest Rally Point
    // @Range: 0 1
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FRAME("RET_RALLY",   8,  AC_Fence,   _ret_rally,       0, AP_PARAM_FRAME_PLANE),

    // @Param{Plane}: RET_ALT
    // @DisplayName: Fence Return Altitude
    // @Description: Altitude the vehicle will transit to when a fence breach occurs
    // @Units: m
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FRAME("RET_ALT",   9,  AC_Fence,   _ret_altitude,       0.0f, AP_PARAM_FRAME_PLANE),

    // @Param{Plane}: AUTOENABLE
    // @DisplayName: Fence Auto-Enable
    // @Description: Auto-enable of fence
    // @Values: 0:AutoEnableOff,1:AutoEnableOnTakeoff,2:AutoEnableDisableFloorOnLanding,3:AutoEnableOnlyWhenArmed
    // @Range: 0 3
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FRAME("AUTOENABLE", 10, AC_Fence, _auto_enabled, static_cast<uint8_t>(AutoEnable::ALWAYS_DISABLED), AP_PARAM_FRAME_PLANE),

    AP_GROUPEND
};

/// Default constructor.
AC_Fence::AC_Fence()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Fence must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

/// enable the Fence code generally; a master switch for all fences
void AC_Fence::enable(bool value)
{
    if (_enabled && !value) {
        AP::logger().Write_Event(LogEvent::FENCE_DISABLE);
    } else if (!_enabled && value) {
        AP::logger().Write_Event(LogEvent::FENCE_ENABLE);
    }
    _enabled.set(value);
    if (!value) {
        clear_breach(AC_FENCE_TYPE_ALT_MIN | AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON);
        disable_floor();
    } else {
        enable_floor();
    }
}

/// enable/disable fence floor only
void AC_Fence::enable_floor()
{
    if (!_floor_enabled) {
        // Floor is currently disabled, enable it
        AP::logger().Write_Event(LogEvent::FENCE_FLOOR_ENABLE);
    }
    _floor_enabled = true;
}

void AC_Fence::disable_floor()
{
    if (_floor_enabled) {
        // Floor is currently enabled, disable it
        AP::logger().Write_Event(LogEvent::FENCE_FLOOR_DISABLE);
    }
    _floor_enabled = false;
    clear_breach(AC_FENCE_TYPE_ALT_MIN);
}

/*
  called when an auto-takeoff is complete
*/
void AC_Fence::auto_enable_fence_after_takeoff(void)
{
    switch(auto_enabled()) {
        case AC_Fence::AutoEnable::ALWAYS_ENABLED:
        case AC_Fence::AutoEnable::ENABLE_DISABLE_FLOOR_ONLY:
            enable(true);
            gcs().send_text(MAV_SEVERITY_NOTICE, "Fence enabled (auto enabled)");
            break;
        default:
            // fence does not auto-enable in other takeoff conditions
            break;
    }
}

/*
  called when performing an auto landing
 */
void AC_Fence::auto_disable_fence_for_landing(void)
{
    switch (auto_enabled()) {
        case AC_Fence::AutoEnable::ALWAYS_ENABLED:
            enable(false);
            gcs().send_text(MAV_SEVERITY_NOTICE, "Fence disabled (auto disable)");
            break;
        case AC_Fence::AutoEnable::ENABLE_DISABLE_FLOOR_ONLY:
            disable_floor();
            gcs().send_text(MAV_SEVERITY_NOTICE, "Fence floor disabled (auto disable)");
            break;
        default:
            // fence does not auto-disable in other landing conditions
            break;
    }
}

bool AC_Fence::present() const
{
    const auto enabled_fences = _enabled_fences.get();
    // A fence is present if any of the conditions are true.
    //   * tin can (circle) is enabled
    //   * min or max alt is enabled
    //   * polygon fences are enabled and any fence has been uploaded
    if (enabled_fences & AC_FENCE_TYPE_CIRCLE ||
        enabled_fences & AC_FENCE_TYPE_ALT_MIN ||
        enabled_fences & AC_FENCE_TYPE_ALT_MAX ||
        ((enabled_fences & AC_FENCE_TYPE_POLYGON) && _poly_loader.total_fence_count() > 0)) {
        return true;
    }

    return false;
}

/// get_enabled_fences - returns bitmask of enabled fences
uint8_t AC_Fence::get_enabled_fences() const
{
    if (!_enabled && !_auto_enabled) {
        return 0;
    }
    return _enabled_fences;
}

// additional checks for the polygon fence:
bool AC_Fence::pre_arm_check_polygon(const char* &fail_msg) const
{
    if (!(_enabled_fences & AC_FENCE_TYPE_POLYGON)) {
        // not enabled; all good
        return true;
    }

    if (! _poly_loader.loaded()) {
        fail_msg = "Fences invalid";
        return false;
    }

    if (!_poly_loader.check_inclusion_circle_margin(_margin)) {
        fail_msg = "Margin is less than inclusion circle radius";
        return false;
    }

    return true;
}

// additional checks for the circle fence:
bool AC_Fence::pre_arm_check_circle(const char* &fail_msg) const
{
    if (_circle_radius < 0) {
        fail_msg = "Invalid FENCE_RADIUS value";
        return false;
    }
    if (_circle_radius < _margin) {
        fail_msg = "FENCE_MARGIN is less than FENCE_RADIUS";
        return false;
    }

    return true;
}

// additional checks for the alt fence:
bool AC_Fence::pre_arm_check_alt(const char* &fail_msg) const
{
    if (_alt_max < 0.0f) {
        fail_msg = "Invalid FENCE_ALT_MAX value";
        return false;
    }

    if (_alt_min < -100.0f) {
        fail_msg = "Invalid FENCE_ALT_MIN value";
        return false;
    }
    return true;
}


/// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
bool AC_Fence::pre_arm_check(const char* &fail_msg) const
{
    fail_msg = nullptr;

    // if fences are enabled but none selected fail pre-arm check
    if (enabled() && !present()) {
        fail_msg = "Fences enabled, but none selected";
        return false;
    }

    // if not enabled or not fence set-up always return true
    if ((!_enabled && !_auto_enabled) || !_enabled_fences) {
        return true;
    }

    // if we have horizontal limits enabled, check we can get a
    // relative position from the AHRS
    if ((_enabled_fences & AC_FENCE_TYPE_CIRCLE) ||
        (_enabled_fences & AC_FENCE_TYPE_POLYGON)) {
        Vector2f position;
        if (!AP::ahrs().get_relative_position_NE_home(position)) {
            fail_msg = "Fence requires position";
            return false;
        }
    }

    if (!pre_arm_check_polygon(fail_msg)) {
        return false;
    }

    if (!pre_arm_check_circle(fail_msg)) {
        return false;
    }

    if (!pre_arm_check_alt(fail_msg)) {
        return false;
    }

    // check no limits are currently breached
    if (_breached_fences) {
        fail_msg =  "vehicle outside fence";
        return false;
    }

    // validate FENCE_MARGIN parameter range
    if (_margin < 0.0f) {
        fail_msg = "Invalid FENCE_MARGIN value";
        return false;
    }

    if (_alt_max < _alt_min) {
        fail_msg =  "FENCE_ALT_MAX < FENCE_ALT_MIN";
        return false;
    }

    if (_alt_max - _alt_min <= 2.0f * _margin) {
        fail_msg =  "FENCE_MARGIN too big";
        return false;
    }

    // if we got this far everything must be ok
    return true;
}

/// returns true if we have freshly breached the maximum altitude
/// fence; also may set up a fallback fence which, if breached, will
/// cause the altitude fence to be freshly breached
bool AC_Fence::check_fence_alt_max()
{
    // altitude fence check
    if (!(_enabled_fences & AC_FENCE_TYPE_ALT_MAX)) {
        // not enabled; no breach
        return false;
    }

    AP::ahrs().get_relative_position_D_home(_curr_alt);
    _curr_alt = -_curr_alt; // translate Down to Up

    // check if we are over the altitude fence
    if (_curr_alt >= _alt_max) {

        // record distance above breach
        _alt_max_breach_distance = _curr_alt - _alt_max;

        // check for a new breach or a breach of the backup fence
        if (!(_breached_fences & AC_FENCE_TYPE_ALT_MAX) ||
            (!is_zero(_alt_max_backup) && _curr_alt >= _alt_max_backup)) {

            // new breach
            record_breach(AC_FENCE_TYPE_ALT_MAX);

            // create a backup fence 20m higher up
            _alt_max_backup = _curr_alt + AC_FENCE_ALT_MAX_BACKUP_DISTANCE;
            // new breach
            return true;
        }
        // old breach
        return false;
    }

    // not breached

    // clear max alt breach if present
    if ((_breached_fences & AC_FENCE_TYPE_ALT_MAX) != 0) {
        clear_breach(AC_FENCE_TYPE_ALT_MAX);
        _alt_max_backup = 0.0f;
        _alt_max_breach_distance = 0.0f;
    }

    return false;
}

/// returns true if we have freshly breached the minimum altitude
/// fence; also may set up a fallback fence which, if breached, will
/// cause the altitude fence to be freshly breached
bool AC_Fence::check_fence_alt_min()
{
    // altitude fence check
    if (!(_enabled_fences & AC_FENCE_TYPE_ALT_MIN)) {
        // not enabled; no breach
        return false;
    }

    AP::ahrs().get_relative_position_D_home(_curr_alt);
    _curr_alt = -_curr_alt; // translate Down to Up

    // check if we are under the altitude fence
    if (_curr_alt <= _alt_min) {

        // record distance below breach
        _alt_min_breach_distance = _alt_min - _curr_alt;

        // check for a new breach or a breach of the backup fence
        if (!(_breached_fences & AC_FENCE_TYPE_ALT_MIN) ||
            (!is_zero(_alt_min_backup) && _curr_alt <= _alt_min_backup)) {

            // new breach
            record_breach(AC_FENCE_TYPE_ALT_MIN);

            // create a backup fence 20m lower down
            _alt_min_backup = _curr_alt - AC_FENCE_ALT_MIN_BACKUP_DISTANCE;
            // new breach
            return true;
        }
        // old breach
        return false;
    }

    // not breached

    // clear min alt breach if present
    if ((_breached_fences & AC_FENCE_TYPE_ALT_MIN) != 0) {
        clear_breach(AC_FENCE_TYPE_ALT_MIN);
        _alt_min_backup = 0.0f;
        _alt_min_breach_distance = 0.0f;
    }

    return false;
}

// check_fence_polygon - returns true if the poly fence is freshly
// breached.  That includes being inside exclusion zones and outside
// inclusions zones
bool AC_Fence::check_fence_polygon()
{
    const bool was_breached = _breached_fences & AC_FENCE_TYPE_POLYGON;
    const bool breached = ((_enabled_fences & AC_FENCE_TYPE_POLYGON) &&
                           _poly_loader.breached());
    if (breached) {
        if (!was_breached) {
            record_breach(AC_FENCE_TYPE_POLYGON);
            return true;
        }
        return false;
    }
    if (was_breached) {
        clear_breach(AC_FENCE_TYPE_POLYGON);
    }
    return false;
}

/// check_fence_circle - returns true if the circle fence (defined via
/// parameters) has been freshly breached.  May also set up a backup
/// fence outside the fence and return a fresh breach if that backup
/// fence is breaced.
bool AC_Fence::check_fence_circle()
{
    if (!(_enabled_fences & AC_FENCE_TYPE_CIRCLE)) {
        // not enabled; no breach
        return false;
    }

    Vector2f home;
    if (AP::ahrs().get_relative_position_NE_home(home)) {
        // we (may) remain breached if we can't update home
        _home_distance = home.length();
    }

    // check if we are outside the fence
    if (_home_distance >= _circle_radius) {

        // record distance outside the fence
        _circle_breach_distance = _home_distance - _circle_radius;

        // check for a new breach or a breach of the backup fence
        if (!(_breached_fences & AC_FENCE_TYPE_CIRCLE) ||
            (!is_zero(_circle_radius_backup) && _home_distance >= _circle_radius_backup)) {
            // new breach
            // create a backup fence 20m further out
            record_breach(AC_FENCE_TYPE_CIRCLE);
            _circle_radius_backup = _home_distance + AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE;
            return true;
        }
        return false;
    }

    // not currently breached

    // clear circle breach if present
    if (_breached_fences & AC_FENCE_TYPE_CIRCLE) {
        clear_breach(AC_FENCE_TYPE_CIRCLE);
        _circle_radius_backup = 0.0f;
        _circle_breach_distance = 0.0f;
    }

    return false;
}


/// check - returns bitmask of fence types breached (if any)
uint8_t AC_Fence::check()
{
    uint8_t ret = 0;

    // return immediately if disabled
    if ((!_enabled && !_auto_enabled) || !_enabled_fences) {
        return 0;
    }

    // clear any breach from a non-enabled fence
    clear_breach(~_enabled_fences);

    // check if pilot is attempting to recover manually
    if (_manual_recovery_start_ms != 0) {
        // we ignore any fence breaches during the manual recovery period which is about 10 seconds
        if ((AP_HAL::millis() - _manual_recovery_start_ms) < AC_FENCE_MANUAL_RECOVERY_TIME_MIN) {
            return 0;
        }
        // recovery period has passed so reset manual recovery time
        // and continue with fence breach checks
        _manual_recovery_start_ms = 0;
    }

    // maximum altitude fence check
    if (check_fence_alt_max()) {
        ret |= AC_FENCE_TYPE_ALT_MAX;
    }

    // minimum altitude fence check
    if (_floor_enabled && check_fence_alt_min()) {
        ret |= AC_FENCE_TYPE_ALT_MIN;
    }

    // circle fence check
    if (check_fence_circle()) {
        ret |= AC_FENCE_TYPE_CIRCLE;
    }

    // polygon fence check
    if (check_fence_polygon()) {
        ret |= AC_FENCE_TYPE_POLYGON;
    }

    // return any new breaches that have occurred
    return ret;
}

// returns true if the destination is within fence (used to reject waypoints outside the fence)
bool AC_Fence::check_destination_within_fence(const Location& loc)
{
    // Altitude fence check - Fence Ceiling
    if ((get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX)) {
        int32_t alt_above_home_cm;
        if (loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_above_home_cm)) {
            if ((alt_above_home_cm * 0.01f) > _alt_max) {
                return false;
            }
        }
    }

    // Altitude fence check - Fence Floor
    if ((get_enabled_fences() & AC_FENCE_TYPE_ALT_MIN)) {
        int32_t alt_above_home_cm;
        if (loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_above_home_cm)) {
            if ((alt_above_home_cm * 0.01f) < _alt_min) {
                return false;
            }
        }
    }

    // Circular fence check
    if ((get_enabled_fences() & AC_FENCE_TYPE_CIRCLE)) {
        if (AP::ahrs().get_home().get_distance(loc) > _circle_radius) {
            return false;
        }
    }

    // polygon fence check
    if ((get_enabled_fences() & AC_FENCE_TYPE_POLYGON)) {
        if (_poly_loader.breached(loc)) {
            return false;
        }
    }

    return true;
}

/// record_breach - update breach bitmask, time and count
void AC_Fence::record_breach(uint8_t fence_type)
{
    // if we haven't already breached a limit, update the breach time
    if (!_breached_fences) {
        const uint32_t now = AP_HAL::millis();
        _breach_time = now;

        // emit a message indicated we're newly-breached, but not too often
        if (now - _last_breach_notify_sent_ms > 1000) {
            _last_breach_notify_sent_ms = now;
            gcs().send_message(MSG_FENCE_STATUS);
        }
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
    _breached_fences &= ~fence_type;
}

/// get_breach_distance - returns maximum distance in meters outside
/// of the given fences.  fence_type is a bitmask here.
float AC_Fence::get_breach_distance(uint8_t fence_type) const
{
    float max = 0.0f;

    if (fence_type & AC_FENCE_TYPE_ALT_MAX) {
        max = MAX(_alt_max_breach_distance, max);
    }
    if (fence_type & AC_FENCE_TYPE_ALT_MIN) {
        max = MAX(_alt_min_breach_distance, max);
    }
    if (fence_type & AC_FENCE_TYPE_CIRCLE) {
        max = MAX(_circle_breach_distance, max);
    }
    return max;
}

/// manual_recovery_start - caller indicates that pilot is re-taking manual control so fence should be disabled for 10 seconds
///     has no effect if no breaches have occurred
void AC_Fence::manual_recovery_start()
{
    // return immediate if we haven't breached a fence
    if (!_breached_fences) {
        return;
    }

    // record time pilot began manual recovery
    _manual_recovery_start_ms = AP_HAL::millis();
}

// methods for mavlink SYS_STATUS message (send_sys_status)
bool AC_Fence::sys_status_present() const
{
    return present();
}

bool AC_Fence::sys_status_enabled() const
{
    if (!sys_status_present()) {
        return false;
    }
    if (_action == AC_FENCE_ACTION_REPORT_ONLY) {
        return false;
    }
    // Fence is only enabled when the flag is enabled
    return _enabled;
}

bool AC_Fence::sys_status_failed() const
{
    if (!sys_status_present()) {
        // not failed if not present; can fail if present but not enabled
        return false;
    }
    if (get_breaches() != 0) {
        return true;
    }
    return false;
}

AC_PolyFence_loader &AC_Fence::polyfence()
{
    return _poly_loader;
}
const AC_PolyFence_loader &AC_Fence::polyfence() const
{
    return _poly_loader;
}


#else  // build type is not appropriate; provide a dummy implementation:
const AP_Param::GroupInfo AC_Fence::var_info[] = { AP_GROUPEND };

AC_Fence::AC_Fence() {};

void AC_Fence::enable(bool value) {};

void AC_Fence::disable_floor() {};

void AC_Fence::auto_enable_fence_after_takeoff() {};
void AC_Fence::auto_disable_fence_for_landing() {};

bool AC_Fence::present() const { return false; }

uint8_t AC_Fence::get_enabled_fences() const { return 0; }

bool AC_Fence::pre_arm_check(const char* &fail_msg) const  { return true; }

uint8_t AC_Fence::check() { return 0; }
bool AC_Fence::check_destination_within_fence(const Location& loc) { return true; }
float AC_Fence::get_breach_distance(uint8_t fence_type) const { return 0.0; }

void AC_Fence::manual_recovery_start() {}

bool AC_Fence::sys_status_present() const { return false; }
bool AC_Fence::sys_status_enabled() const { return false; }
bool AC_Fence::sys_status_failed() const { return false; }

AC_PolyFence_loader &AC_Fence::polyfence()
{
    return _poly_loader;
}
const AC_PolyFence_loader &AC_Fence::polyfence() const
{
    return _poly_loader;
}

#endif // #if AC_FENCE_DUMMY_METHODS_ENABLED

// singleton instance
AC_Fence *AC_Fence::_singleton;

namespace AP
{

AC_Fence *fence()
{
    return AC_Fence::get_singleton();
}

}

#endif // AP_FENCE_ENABLED
