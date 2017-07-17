#include <AP_HAL/AP_HAL.h>
#include "AC_Fence.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

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
    // @Values: 0:None,1:Altitude,2:Circle,3:Altitude and Circle,4:Polygon,5:Altitude and Polygon,6:Circle and Polygon,7:All
    // @Bitmask: 0:Altitude,1:Circle,2:Polygon
    // @User: Standard
    AP_GROUPINFO("TYPE",        1,  AC_Fence,   _enabled_fences,  AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON),

    // @Param: ACTION
    // @DisplayName: Fence Action
    // @Description: What action should be taken when fence is breached
    // @Values: 0:Report Only,1:RTL or Land
    // @User: Standard
    AP_GROUPINFO("ACTION",      2,  AC_Fence,   _action,        AC_FENCE_ACTION_RTL_AND_LAND),

    // @Param: ALT_MAX
    // @DisplayName: Fence Maximum Altitude
    // @Description: Maximum altitude allowed before geofence triggers
    // @Units: m
    // @Range: 10 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FRAME("ALT_MAX", 3, AC_Fence, _alt_max, AC_FENCE_ALT_MAX_DEFAULT, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_SUB | AP_PARAM_FRAME_TRICOPTER | AP_PARAM_FRAME_HELI),

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

    // @Param: ALT_MIN
    // @DisplayName: Fence Minimum Altitude
    // @Description: Minimum altitude allowed before geofence triggers
    // @Units: m
    // @Range: -100 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FRAME("ALT_MIN",     7,  AC_Fence,   _alt_min,       AC_FENCE_ALT_MIN_DEFAULT, AP_PARAM_FRAME_SUB),

    AP_GROUPEND
};

/// Default constructor.
AC_Fence::AC_Fence(const AP_AHRS_NavEKF& ahrs) :
    _ahrs(ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_Fence::enable(bool value)
{
    _enabled = value;
    if (!value) {
        clear_breach(AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON);
    }
}

/// get_enabled_fences - returns bitmask of enabled fences
uint8_t AC_Fence::get_enabled_fences() const
{
    if (!_enabled) {
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

    if (!_boundary_valid) {
        fail_msg = "Polygon boundary invalid";
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
    return true;
}

// additional checks for the alt fence:
bool AC_Fence::pre_arm_check_alt(const char* &fail_msg) const
{
    if (_alt_max < 0.0f) {
        fail_msg = "Invalid FENCE_ALT_MAX value";
        return false;
    }
    return true;
}


/// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
bool AC_Fence::pre_arm_check(const char* &fail_msg) const
{
    fail_msg = nullptr;

    // if not enabled or not fence set-up always return true
    if (!_enabled || !_enabled_fences) {
        return true;
    }

    // check no limits are currently breached
    if (_breached_fences) {
        fail_msg =  "vehicle outside fence";
        return false;
    }

    // if we have horizontal limits enabled, check we can get a
    // relative position from the EKF
    if ((_enabled_fences & AC_FENCE_TYPE_CIRCLE) ||
        (_enabled_fences & AC_FENCE_TYPE_POLYGON)) {
        Vector2f position;
        if (!_ahrs.get_relative_position_NE_home(position)) {
            fail_msg = "fence requires position";
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

    // if we got this far everything must be ok
    return true;
}

bool AC_Fence::check_fence_alt_max()
{
    // altitude fence check
    if (!(_enabled_fences & AC_FENCE_TYPE_ALT_MAX)) {
        // not enabled; no breach
        return false;
    }

    _ahrs.get_relative_position_D_home(_curr_alt);
    _curr_alt = -_curr_alt; // translate Down to Up

    // check if we are over the altitude fence
    if(_curr_alt >= _alt_max) {

        // record distance above breach
        _alt_max_breach_distance = _curr_alt - _alt_max;

        // check for a new breach or a breach of the backup fence
        if (!(_breached_fences & AC_FENCE_TYPE_ALT_MAX) ||
            (!is_zero(_alt_max_backup) && _curr_alt >= _alt_max_backup)) {

            // new breach
            record_breach(AC_FENCE_TYPE_ALT_MAX);

            // create a backup fence 20m higher up
            _alt_max_backup = _curr_alt + AC_FENCE_ALT_MAX_BACKUP_DISTANCE;
            // new breach:
            return true;
        }
        // old breach:
        return false;
    }

    // not breached

    // clear alt breach if present
    if ((_breached_fences & AC_FENCE_TYPE_ALT_MAX) != 0) {
        clear_breach(AC_FENCE_TYPE_ALT_MAX);
        _alt_max_backup = 0.0f;
        _alt_max_breach_distance = 0.0f;
    }

    return false;
}

// check_fence_polygon - returns true if the polygon fence is freshly breached
bool AC_Fence::check_fence_polygon()
{
    if (!(_enabled_fences & AC_FENCE_TYPE_POLYGON)) {
        // not enabled; no breach
        return false;
    }

    // check consistency of number of points
    if (_boundary_num_points != _total) {
        // Fence is currently not completely loaded.  Can't breach it?!
        _boundary_loaded = false;
        load_polygon_from_eeprom();
        return false;
    }
    if (!_boundary_valid) {
        // fence isn't valid - can't breach it?!
        return false;
    }

    // check if vehicle is outside the polygon fence
    Vector2f position;
    if (!_ahrs.get_relative_position_NE_origin(position)) {
        // we have no idea where we are; can't breach the fence
        return false;
    }

    position = position * 100.0f;  // m to cm
    if (_poly_loader.boundary_breached(position, _boundary_num_points, _boundary, true)) {
        // check if this is a new breach
        if (_breached_fences & AC_FENCE_TYPE_POLYGON) {
            // not a new breach
            return false;
        }
        // record that we have breached the polygon
        record_breach(AC_FENCE_TYPE_POLYGON);
        return true;
    }

    // inside boundary; clear breach if present
    if (_breached_fences & AC_FENCE_TYPE_POLYGON) {
        clear_breach(AC_FENCE_TYPE_POLYGON);
    }

    return false;
}

bool AC_Fence::check_fence_circle()
{
    if (!(_enabled_fences & AC_FENCE_TYPE_CIRCLE)) {
        // not enabled; no breach
        return false;
    }

    Vector2f home;
    if (_ahrs.get_relative_position_NE_home(home)) {
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
    if (!_enabled || !_enabled_fences) {
        return 0;
    }

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
bool AC_Fence::check_destination_within_fence(const Location_Class& loc)
{
    // Altitude fence check
    if ((get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX)) {
        int32_t alt_above_home_cm;
        if (loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_HOME, alt_above_home_cm)) {
            if ((alt_above_home_cm * 0.01f) > _alt_max) {
                return false;
            }
        }
    }

    // Circular fence check
    if ((get_enabled_fences() & AC_FENCE_TYPE_CIRCLE)) {
        if ((get_distance_cm(_ahrs.get_home(), loc) * 0.01f) > _circle_radius) {
            return false;
        }
    }

    // polygon fence check
    if ((get_enabled_fences() & AC_FENCE_TYPE_POLYGON) && _boundary_num_points > 0) {
        // check ekf has a good location
        Vector2f posNE;
        if (loc.get_vector_xy_from_origin_NE(posNE)) {
            if (_poly_loader.boundary_breached(posNE, _boundary_num_points, _boundary, true)) {
                return false;
            }
        }
    }

    return true;
}

/// record_breach - update breach bitmask, time and count
void AC_Fence::record_breach(uint8_t fence_type)
{
    // if we haven't already breached a limit, update the breach time
    if (!_breached_fences) {
        _breach_time = AP_HAL::millis();
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
            return MAX(_alt_max_breach_distance,_circle_breach_distance);
    }

    // we don't recognise the fence type so just return 0
    return 0;
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

/// returns pointer to array of polygon points and num_points is filled in with the total number
Vector2f* AC_Fence::get_polygon_points(uint16_t& num_points) const
{
    num_points = _boundary_num_points;
    return _boundary;
}

/// returns true if we've breached the polygon boundary.  simple passthrough to underlying _poly_loader object
bool AC_Fence::boundary_breached(const Vector2f& location, uint16_t num_points, const Vector2f* points) const
{
    return _poly_loader.boundary_breached(location, num_points, points, true);
}

/// handler for polygon fence messages with GCS
void AC_Fence::handle_msg(GCS_MAVLINK &link, mavlink_message_t* msg)
{
    // exit immediately if null message
    if (msg == nullptr) {
        return;
    }

    switch (msg->msgid) {
        // receive a fence point from GCS and store in EEPROM
        case MAVLINK_MSG_ID_FENCE_POINT: {
            mavlink_fence_point_t packet;
            mavlink_msg_fence_point_decode(msg, &packet);
            if (!check_latlng(packet.lat,packet.lng)) {
                link.send_text(MAV_SEVERITY_WARNING, "Invalid fence point, lat or lng too large");
            } else {
                Vector2l point;
                point.x = packet.lat*1.0e7f;
                point.y = packet.lng*1.0e7f;
                if (!_poly_loader.save_point_to_eeprom(packet.idx, point)) {
                    link.send_text(MAV_SEVERITY_WARNING, "Failed to save polygon point, too many points?");
                } else {
                    // trigger reload of points
                    _boundary_loaded = false;
                }
            }
            break;
        }

        // send a fence point to GCS
        case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
            mavlink_fence_fetch_point_t packet;
            mavlink_msg_fence_fetch_point_decode(msg, &packet);
            // attempt to retrieve from eeprom
            Vector2l point;
            if (_poly_loader.load_point_from_eeprom(packet.idx, point)) {
                mavlink_msg_fence_point_send_buf(msg, link.get_chan(), msg->sysid, msg->compid, packet.idx, _total, point.x*1.0e-7f, point.y*1.0e-7f);
            } else {
                link.send_text(MAV_SEVERITY_WARNING, "Bad fence point");
            }
            break;
        }

        default:
            // do nothing
            break;
    }
}

/// load polygon points stored in eeprom into boundary array and perform validation
bool AC_Fence::load_polygon_from_eeprom(bool force_reload)
{
    // exit immediately if already loaded
    if (_boundary_loaded && !force_reload) {
        return true;
    }

    // check if we need to create array
    if (!_boundary_create_attempted) {
        _boundary = (Vector2f *)_poly_loader.create_point_array(sizeof(Vector2f));
        _boundary_create_attempted = true;
    }

    // exit if we could not allocate RAM for the boundary
    if (_boundary == nullptr) {
        return false;
    }

    // get current location from EKF
    Location temp_loc;
    if (!_ahrs.get_location(temp_loc)) {
        return false;
    }
    struct Location ekf_origin {};
    _ahrs.get_origin(ekf_origin);

    // sanity check total
    _total = constrain_int16(_total, 0, _poly_loader.max_points());

    // load each point from eeprom
    Vector2l temp_latlon;
    for (uint16_t index=0; index<_total; index++) {
        // load boundary point as lat/lon point
        _poly_loader.load_point_from_eeprom(index, temp_latlon);
        // move into location structure and convert to offset from ekf origin
        temp_loc.lat = temp_latlon.x;
        temp_loc.lng = temp_latlon.y;
        _boundary[index] = location_diff(ekf_origin, temp_loc) * 100.0f;
    }
    _boundary_num_points = _total;
    _boundary_loaded = true;

    // update validity of polygon
    _boundary_valid = _poly_loader.boundary_valid(_boundary_num_points, _boundary, true);

    return true;
}

// methods for mavlink SYS_STATUS message (send_extended_status1)
bool AC_Fence::sys_status_present() const
{
    return _enabled;
}

bool AC_Fence::sys_status_enabled() const
{
    if (!sys_status_present()) {
        return false;
    }
    if (_action == AC_FENCE_ACTION_REPORT_ONLY) {
        return false;
    }
    return true;
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
    if (_enabled_fences & AC_FENCE_TYPE_POLYGON) {
        if (!_boundary_valid) {
            return true;
        }
    }
    if (_enabled_fences & AC_FENCE_TYPE_CIRCLE) {
        if (_circle_radius < 0) {
            return true;
        }
    }
    if (_enabled_fences & AC_FENCE_TYPE_ALT_MAX) {
        if (_alt_max < 0.0f) {
            return true;
        }
    }
    if ((_enabled_fences & AC_FENCE_TYPE_CIRCLE) ||
        (_enabled_fences & AC_FENCE_TYPE_POLYGON)) {
        Vector2f position;
        if (!_ahrs.get_relative_position_NE_home(position)) {
            // both these fence types require position
            return true;
        }
    }

    return false;
}
