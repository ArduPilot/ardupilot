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
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_Rover)
#define AC_FENCE_TYPE_DEFAULT AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AC_FENCE_TYPE_DEFAULT AC_FENCE_TYPE_POLYGON
#else
#define AC_FENCE_TYPE_DEFAULT AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON
#endif

// default boundaries
#define AC_FENCE_ALT_MAX_DEFAULT                    100.0f  // default max altitude is 100m
#define AC_FENCE_ALT_MIN_DEFAULT                    -10.0f  // default maximum depth in meters
#define AC_FENCE_CIRCLE_RADIUS_DEFAULT              300.0f  // default circular fence radius is 300m
#define AC_FENCE_ALT_MAX_BACKUP_DISTANCE            20.0f   // after fence is broken we recreate the fence 20m further up
#define AC_FENCE_ALT_MIN_BACKUP_DISTANCE            20.0f   // after fence is broken we recreate the fence 20m further down
#define AC_FENCE_MARGIN_DEFAULT                     2.0f    // default distance in meters that autopilot's should maintain from the fence to avoid a breach
#define AC_FENCE_MANUAL_RECOVERY_TIME_MIN           10000   // pilot has 10seconds to recover during which time the autopilot will not attempt to re-take control

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE     100.0   // after fence is broken we recreate the fence 100m further out
#define AC_FENCE_OPTIONS_DEFAULT                   OPTIONS::DISABLE_MODE_CHANGE
#else
#define AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE      20.0   // after fence is broken we recreate the fence 20m further out
#define AC_FENCE_OPTIONS_DEFAULT                   0
#endif

//#define AC_FENCE_DEBUG

const AP_Param::GroupInfo AC_Fence::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Fence enable/disable
    // @Description: Allows you to enable (1) or disable (0) the fence functionality. Fences can still be enabled and disabled via mavlink or an RC option, but these changes are not persisted.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",      0,  AC_Fence,   _enabled,   0),

    // @Param: TYPE
    // @DisplayName: Fence Type
    // @Description: Configured fence types held as bitmask. Max altitide, Circle and Polygon fences will be immediately enabled if configured. Min altitude fence will only be enabled once the minimum altitude is reached.
    // @Bitmask{Rover}: 1:Circle Centered on Home,2:Inclusion/Exclusion Circles+Polygons
    // @Bitmask{Copter, Plane, Sub}: 0:Max altitude,1:Circle Centered on Home,2:Inclusion/Exclusion Circles+Polygons,3:Min altitude
    // @User: Standard
    AP_GROUPINFO("TYPE",        1,  AC_Fence,   _configured_fences,  AC_FENCE_TYPE_DEFAULT),

    // @Param: ACTION
    // @DisplayName: Fence Action
    // @Description: What action should be taken when fence is breached
    // @Values{Copter}: 0:Report Only,1:RTL or Land,2:Always Land,3:SmartRTL or RTL or Land,4:Brake or Land,5:SmartRTL or Land
    // @Values{Rover}: 0:Report Only,1:RTL or Hold,2:Hold,3:SmartRTL or RTL or Hold,4:SmartRTL or Hold
    // @Values{Plane}: 0:Report Only,1:RTL,6:Guided,7:GuidedThrottlePass
    // @Values: 0:Report Only,1:RTL or Land
    // @User: Standard
    AP_GROUPINFO("ACTION",      2,  AC_Fence,   _action,        static_cast<uint8_t>(AC_Fence::Action::RTL_AND_LAND)),

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

    // @Param{Plane, Copter}: AUTOENABLE
    // @DisplayName: Fence Auto-Enable
    // @Description: Auto-enable of fences. AutoEnableOnTakeoff enables all configured fences, except the minimum altitude fence (which is enabled when the minimum altitude is reached), after autotakeoffs reach altitude. During autolandings the fences will be disabled.  AutoEnableDisableFloorOnLanding enables all configured fences, except the minimum altitude fence (which is enabled when the minimum altitude is reached), after autotakeoffs reach altitude. During autolandings only the Minimum Altitude fence will be disabled. AutoEnableOnlyWhenArmed enables all configured fences on arming, except the minimum altitude fence (which is enabled when the minimum altitude is reached), but no fences are disabled during autolandings. However, fence breaches are ignored while executing prior breach recovery actions which may include autolandings.
    // @Values{Plane, Copter}: 0:AutoEnableOff,1:AutoEnableOnTakeoff,2:AutoEnableDisableFloorOnLanding,3:AutoEnableOnlyWhenArmed
    // @Range: 0 3
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FRAME("AUTOENABLE", 10, AC_Fence, _auto_enabled, static_cast<uint8_t>(AutoEnable::ALWAYS_DISABLED), AP_PARAM_FRAME_PLANE | AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_TRICOPTER | AP_PARAM_FRAME_HELI),

    // @Param{Plane, Copter}: OPTIONS
    // @DisplayName: Fence options
    // @Description: When bit 0 is set sisable mode change following fence action until fence breach is cleared. When bit 1 is set the allowable flight areas is the union of all polygon and circle fence areas instead of the intersection, which means a fence breach occurs only if you are outside all of the fence areas.
    // @Bitmask: 0:Disable mode change following fence action until fence breach is cleared, 1:Allow union of inclusion areas
    // @User: Standard
    AP_GROUPINFO_FRAME("OPTIONS", 11, AC_Fence, _options, static_cast<uint16_t>(AC_FENCE_OPTIONS_DEFAULT), AP_PARAM_FRAME_PLANE | AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_TRICOPTER | AP_PARAM_FRAME_HELI),

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
    if (_enabled) {
        _enabled_fences = _configured_fences.get() & ~AC_FENCE_TYPE_ALT_MIN;
    }
}

// get a user-friendly list of fences
void AC_Fence::get_fence_names(uint8_t fences, ExpandingString& msg)
{
    if (!fences) {
        return;
    }
    static const char* FENCE_NAMES[] = {
        "Max Alt",
        "Circle",
        "Polygon",
        "Min Alt",
    };
    uint8_t i = 0;
    uint8_t nfences = 0;
    while (fences !=0) {
        if (fences & 0x1) {
            if (nfences > 0) {
                if (!(fences & ~1U)) {
                    msg.printf(" and ");
                } else {
                    msg.printf(", ");
                }
            }
            msg.printf("%s", FENCE_NAMES[i]);
            nfences++;
        }
        fences >>= 1;
        i++;
    }
    msg.printf(" fence");
    if (nfences>1) {
        msg.printf("s");
    }
}

// print a message about the passed in fences
void AC_Fence::print_fence_message(const char* message, uint8_t fences) const
{
    if (!fences) {
        return;
    }

    char msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    ExpandingString e(msg, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1);
    AC_Fence::get_fence_names(fences, e);
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "%s %s", e.get_writeable_string(), message);
}

// should be called @10Hz to handle loading from eeprom
void AC_Fence::update()
{
    _poly_loader.update();
    // if someone changes the parameter we want to enable or disable everything
    if (_enabled != _last_enabled || _auto_enabled != _last_auto_enabled) {
        // reset the auto mask since we just reconfigured all of fencing
        _auto_enable_mask = AC_FENCE_ALL_FENCES;
        _last_enabled = _enabled;
        _last_auto_enabled = _auto_enabled;
        if (_enabled) {
            _enabled_fences = _configured_fences.get() & ~AC_FENCE_TYPE_ALT_MIN;
        } else {
            _enabled_fences = 0;
        }
    }
#ifdef AC_FENCE_DEBUG
    static uint32_t last_msg_count = 0;
    if (get_enabled_fences() && last_msg_count++ % 10 == 0) {
        print_fence_message("active", get_enabled_fences());
        print_fence_message("breached", get_breaches());
    }
#endif
}

// enable or disable configured fences present in fence_types
// also updates the bitmask of auto enabled fences if update_auto_mask is true
// returns a bitmask of fences that were changed
uint8_t AC_Fence::enable(bool value, uint8_t fence_types, bool update_auto_mask)
{
    uint8_t fences = _configured_fences.get() & fence_types;
    uint8_t enabled_fences = _enabled_fences;
    if (value) {
        enabled_fences |= fences;
    } else {
        enabled_fences &= ~fences;
    }

    // fences that were manually changed are no longer eligible for auto-enablement or disablement
    if (update_auto_mask) {
        _auto_enable_mask &= ~fences;
    }

    uint8_t fences_to_change = _enabled_fences ^ enabled_fences;

    if (!fences_to_change) {
        return 0;
    }
#if HAL_LOGGING_ENABLED
    AP::logger().Write_Event(value ? LogEvent::FENCE_ENABLE : LogEvent::FENCE_DISABLE);
    if (fences_to_change & AC_FENCE_TYPE_ALT_MAX) {
        AP::logger().Write_Event(value ? LogEvent::FENCE_ALT_MAX_ENABLE : LogEvent::FENCE_ALT_MAX_DISABLE);
    }
    if (fences_to_change & AC_FENCE_TYPE_CIRCLE) {
        AP::logger().Write_Event(value ? LogEvent::FENCE_CIRCLE_ENABLE : LogEvent::FENCE_CIRCLE_DISABLE);
    }
    if (fences_to_change & AC_FENCE_TYPE_ALT_MIN) {
        AP::logger().Write_Event(value ? LogEvent::FENCE_ALT_MIN_ENABLE : LogEvent::FENCE_ALT_MIN_DISABLE);
    }
    if (fences_to_change & AC_FENCE_TYPE_POLYGON) {
        AP::logger().Write_Event(value ? LogEvent::FENCE_POLYGON_ENABLE : LogEvent::FENCE_POLYGON_DISABLE);
    }
#endif

    _enabled_fences = enabled_fences;

    if (!value) {
        clear_breach(fences_to_change);
    }

    return fences_to_change;
}

/// enable/disable fence floor only
void AC_Fence::enable_floor()
{
    enable(true, AC_FENCE_TYPE_ALT_MIN);
}

void AC_Fence::disable_floor()
{
    enable(false, AC_FENCE_TYPE_ALT_MIN);
}

/*
  called on arming
*/
void AC_Fence::auto_enable_fence_on_arming(void)
{
    if (auto_enabled() != AC_Fence::AutoEnable::ONLY_WHEN_ARMED) {
        return;
    }

    const uint8_t fences = enable(true, _auto_enable_mask & ~AC_FENCE_TYPE_ALT_MIN, false);
    print_fence_message("auto-enabled", fences);
}

/*
  called on disarming
*/
void AC_Fence::auto_disable_fence_on_disarming(void)
{
    if (auto_enabled() != AC_Fence::AutoEnable::ONLY_WHEN_ARMED) {
        return;
    }

    const uint8_t fences = enable(false, _auto_enable_mask, false);
    print_fence_message("auto-disabled", fences);
}

/*
  called when an auto-takeoff is complete
*/
void AC_Fence::auto_enable_fence_after_takeoff(void)
{
    if (auto_enabled() != AutoEnable::ENABLE_ON_AUTO_TAKEOFF &&
        auto_enabled() != AutoEnable::ENABLE_DISABLE_FLOOR_ONLY) {
        return;
    }

    const uint8_t fences = enable(true, _auto_enable_mask, false);
    print_fence_message("auto-enabled", fences);
}

// return fences that should be auto-disabled when requested
uint8_t AC_Fence::get_auto_disable_fences(void) const
{
    uint8_t auto_disable = 0;
    switch (auto_enabled()) {
        case AC_Fence::AutoEnable::ENABLE_ON_AUTO_TAKEOFF:
            auto_disable = _auto_enable_mask;
            break;
        case AC_Fence::AutoEnable::ENABLE_DISABLE_FLOOR_ONLY:
        case AC_Fence::AutoEnable::ONLY_WHEN_ARMED:
        default: // when auto disable is not set we still need to disable the altmin fence on landing
            auto_disable = _auto_enable_mask & AC_FENCE_TYPE_ALT_MIN;
            break;
    }
    return auto_disable;
}

uint8_t AC_Fence::present() const
{
    uint8_t mask = AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_ALT_MIN | AC_FENCE_TYPE_ALT_MAX;
    if (_poly_loader.total_fence_count() > 0) {
        mask |= AC_FENCE_TYPE_POLYGON;
    }

    return _configured_fences.get() & mask;
}

/// get_enabled_fences - returns bitmask of enabled fences
uint8_t AC_Fence::get_enabled_fences() const
{
    return _enabled_fences & present();
}

// additional checks for the polygon fence:
bool AC_Fence::pre_arm_check_polygon(char *failure_msg, const uint8_t failure_msg_len) const
{
    if (!(_configured_fences & AC_FENCE_TYPE_POLYGON)) {
        // not enabled; all good
        return true;
    }

    if (! _poly_loader.loaded()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Polygon fence(s) invalid");
        return false;
    }

    if (!_poly_loader.check_inclusion_circle_margin(_margin)) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Polygon fence margin is less than inclusion circle radius");
        return false;
    }

    return true;
}

// additional checks for the circle fence:
bool AC_Fence::pre_arm_check_circle(char *failure_msg, const uint8_t failure_msg_len) const
{
    if (_circle_radius < 0) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Invalid Circle FENCE_RADIUS value");
        return false;
    }
    if (_circle_radius < _margin) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Circle FENCE_MARGIN is less than FENCE_RADIUS");
        return false;
    }

    return true;
}

// additional checks for the alt fence:
bool AC_Fence::pre_arm_check_alt(char *failure_msg, const uint8_t failure_msg_len) const
{
    if (_alt_max < 0.0f) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Invalid FENCE_ALT_MAX value");
        return false;
    }

    if (_alt_min < -100.0f) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Invalid FENCE_ALT_MIN value");
        return false;
    }
    return true;
}


/// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
bool AC_Fence::pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const
{
    // if fences are enabled but none selected fail pre-arm check
    if (_enabled && !present()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Fences enabled, but none selected");
        return false;
    }
    
    // if AUTOENABLE = 1 or 2 warn now, but fail in a later release
    // PARAMETER_CONVERSION - Added: Jul-2024 for ArduPilot-4.6
    if (_auto_enabled == 1 || _auto_enabled == 2) {
        static uint32_t last_autoenable_warn_ms;
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_autoenable_warn_ms > 60000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FENCE_AUTOENABLE is %u, will be removed in 4.7, use 3", unsigned(_auto_enabled));
            last_autoenable_warn_ms = now_ms;
        }
    }

    // if not enabled or not fence set-up always return true
    if ((!enabled() && !_auto_enabled) || !_configured_fences) {
        return true;
    }

    // if we have horizontal limits enabled, check we can get a
    // relative position from the AHRS
    if ((_configured_fences & AC_FENCE_TYPE_CIRCLE) ||
        (_configured_fences & AC_FENCE_TYPE_POLYGON)) {
        Vector2f position;
        if (!AP::ahrs().get_relative_position_NE_home(position)) {
            hal.util->snprintf(failure_msg, failure_msg_len, "Fence requires position");
            return false;
        }
    }

    if (!pre_arm_check_polygon(failure_msg, failure_msg_len)) {
        return false;
    }

    if (!pre_arm_check_circle(failure_msg, failure_msg_len)) {
        return false;
    }

    if (!pre_arm_check_alt(failure_msg, failure_msg_len)) {
        return false;
    }

    // check no limits are currently breached
    if (_breached_fences) {
        char msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
        ExpandingString e(msg, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1);
        AC_Fence::get_fence_names(_breached_fences, e);
        hal.util->snprintf(failure_msg, failure_msg_len, "Vehicle breaching %s", e.get_writeable_string());
        return false;
    }

    // validate FENCE_MARGIN parameter range
    if (_margin < 0.0f) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Invalid FENCE_MARGIN value");
        return false;
    }

    if (_alt_max < _alt_min) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FENCE_ALT_MAX < FENCE_ALT_MIN");
        return false;
    }

    if (_alt_max - _alt_min <= 2.0f * _margin) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FENCE_MARGIN too big");
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
    if (!(get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX)) {
        // not enabled; no breach
        return false;
    }

    float alt;
    AP::ahrs().get_relative_position_D_home(alt);
    _curr_alt = -alt; // translate Down to Up

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
    if (!(get_enabled_fences() & AC_FENCE_TYPE_ALT_MIN)) {
        // not enabled; no breach
        return false;
    }

    float alt;
    AP::ahrs().get_relative_position_D_home(alt);
    _curr_alt = -alt; // translate Down to Up

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


/// auto enable fence floor
bool AC_Fence::auto_enable_fence_floor()
{
    // altitude fence check
    if (!(_configured_fences & AC_FENCE_TYPE_ALT_MIN)       // not configured
        || (get_enabled_fences() & AC_FENCE_TYPE_ALT_MIN)   // already enabled
        || !(_auto_enable_mask & AC_FENCE_TYPE_ALT_MIN)     // has been manually disabled
        || (!_enabled && (auto_enabled() == AC_Fence::AutoEnable::ALWAYS_DISABLED
            || auto_enabled() == AutoEnable::ENABLE_ON_AUTO_TAKEOFF))) {
        // not enabled
        return false;
    }

    float alt;
    AP::ahrs().get_relative_position_D_home(alt);
    _curr_alt = -alt; // translate Down to Up

    // check if we are over the altitude fence
    if (!floor_enabled() && _curr_alt >= _alt_min + _margin) {
        enable(true, AC_FENCE_TYPE_ALT_MIN, false);
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Min Alt fence enabled (auto enable)");
        return true;
    } 

    return false;
}

// check_fence_polygon - returns true if the poly fence is freshly
// breached.  That includes being inside exclusion zones and outside
// inclusions zones
bool AC_Fence::check_fence_polygon()
{
    if (!(get_enabled_fences() & AC_FENCE_TYPE_POLYGON)) {
        // not enabled; no breach
        clear_breach(AC_FENCE_TYPE_POLYGON);
        return false;
    }

    const bool was_breached = _breached_fences & AC_FENCE_TYPE_POLYGON;
    if (_poly_loader.breached()) {
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
/// fence is breached.
bool AC_Fence::check_fence_circle()
{
    if (!(get_enabled_fences() & AC_FENCE_TYPE_CIRCLE)) {
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
            // create a backup fence 20m or 100m further out
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
uint8_t AC_Fence::check(bool disable_auto_fences)
{
    uint8_t ret = 0;
    uint8_t disabled_fences = disable_auto_fences ? get_auto_disable_fences() : 0;
    uint8_t fences_to_disable = disabled_fences & _enabled_fences;

    // clear any breach from a non-enabled fence
    clear_breach(~_configured_fences);
    // clear any breach from disabled fences
    clear_breach(fences_to_disable);

    // report on any fences that were auto-disabled
    if (fences_to_disable) {
        print_fence_message("auto-disabled", fences_to_disable);
    }

    // return immediately if disabled
    if ((!enabled() && !_auto_enabled && !(_configured_fences & AC_FENCE_TYPE_ALT_MIN)) || !_configured_fences) {
        return 0;
    }

    // disable the (temporarily) disabled fences
    enable(false, disabled_fences, false);

    // maximum altitude fence check
    if (!(disabled_fences & AC_FENCE_TYPE_ALT_MAX) && check_fence_alt_max()) {
        ret |= AC_FENCE_TYPE_ALT_MAX;
    }

    // minimum altitude fence check, do this before auto-disabling (e.g. because falling)
    // so that any action can be taken
    if (!(disabled_fences & AC_FENCE_TYPE_ALT_MIN) && check_fence_alt_min()) {
        ret |= AC_FENCE_TYPE_ALT_MIN;
    }

    // auto enable floor unless auto enable on auto takeoff has been set (which means other behaviour is required)
    if (!(disabled_fences & AC_FENCE_TYPE_ALT_MIN)) {
        auto_enable_fence_floor();
    }

    // circle fence check
    if (!(disabled_fences & AC_FENCE_TYPE_CIRCLE) && check_fence_circle()) {
        ret |= AC_FENCE_TYPE_CIRCLE;
    }

    // polygon fence check
    if (!(disabled_fences & AC_FENCE_TYPE_POLYGON) && check_fence_polygon()) {
        ret |= AC_FENCE_TYPE_POLYGON;
    }

    // check if pilot is attempting to recover manually
    // this is done last so that _breached_fences is correct
    if (_manual_recovery_start_ms != 0) {
        // we ignore any fence breaches during the manual recovery period which is about 10 seconds
        if ((AP_HAL::millis() - _manual_recovery_start_ms) < AC_FENCE_MANUAL_RECOVERY_TIME_MIN) {
            return 0;
        }
        // recovery period has passed so reset manual recovery time
        // and continue with fence breach checks
        _manual_recovery_start_ms = 0;
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
            GCS_SEND_MESSAGE(MSG_FENCE_STATUS);
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

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Manual recovery started");
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
    if (static_cast<AC_Fence::Action>(static_cast<uint8_t>(_action)) == AC_Fence::Action::REPORT_ONLY) {
        return false;
    }
    // Fence is only enabled when the flag is enabled
    return enabled();
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

uint8_t AC_Fence::enable(bool value, uint8_t fence_types, bool update_auto_enable) { return 0; }

void AC_Fence::enable_floor() {}
void AC_Fence::disable_floor() {}
void AC_Fence::update() {}

void AC_Fence::auto_enable_fence_after_takeoff() {}
void AC_Fence::auto_enable_fence_on_arming() {}
void AC_Fence::auto_disable_fence_on_disarming() {}

uint8_t AC_Fence::present() const { return 0; }

uint8_t AC_Fence::get_enabled_fences() const { return 0; }

bool AC_Fence::pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const  { return true; }

uint8_t AC_Fence::check(bool disable_auto_fences) { return 0; }
bool AC_Fence::check_destination_within_fence(const Location& loc) { return true; }
float AC_Fence::get_breach_distance(uint8_t fence_type) const { return 0.0; }
void AC_Fence::get_fence_names(uint8_t fences, ExpandingString& msg) { }
void AC_Fence::print_fence_message(const char* msg, uint8_t fences) const {}

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
