#include "Copter.h"

Mode::AutoYaw Mode::auto_yaw;

// roi_yaw - returns heading towards location held in roi
float Mode::AutoYaw::roi_yaw() const
{
    return get_bearing_cd(copter.inertial_nav.get_position(), roi);
}

float Mode::AutoYaw::look_ahead_yaw()
{
    const Vector3f& vel = copter.inertial_nav.get_velocity();
    float speed = norm(vel.x,vel.y);
    // Commanded Yaw to automatically look ahead.
    if (copter.position_ok() && (speed > YAW_LOOK_AHEAD_MIN_SPEED)) {
        _look_ahead_yaw = degrees(atan2f(vel.y,vel.x))*100.0f;
    }
    return _look_ahead_yaw;
}

void Mode::AutoYaw::set_mode_to_default(bool rtl)
{
    set_mode(default_mode(rtl));
}

// default_mode - returns auto_yaw.mode() based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
autopilot_yaw_mode Mode::AutoYaw::default_mode(bool rtl) const
{
    switch (copter.g.wp_yaw_behavior) {

    case WP_YAW_BEHAVIOR_NONE:
        return AUTO_YAW_HOLD;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
        if (rtl) {
            return AUTO_YAW_HOLD;
        } else {
            return AUTO_YAW_LOOK_AT_NEXT_WP;
        }

    case WP_YAW_BEHAVIOR_LOOK_AHEAD:
        return AUTO_YAW_LOOK_AHEAD;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
    default:
        return AUTO_YAW_LOOK_AT_NEXT_WP;
    }
}

// set_mode - sets the yaw mode for auto
void Mode::AutoYaw::set_mode(autopilot_yaw_mode yaw_mode)
{
    // return immediately if no change
    if (_mode == yaw_mode) {
        return;
    }
    _mode = yaw_mode;

    // perform initialisation
    switch (_mode) {

    case AUTO_YAW_LOOK_AT_NEXT_WP:
        // wpnav will initialise heading when wpnav's set_destination method is called
        break;

    case AUTO_YAW_ROI:
        // look ahead until we know otherwise
        break;

    case AUTO_YAW_FIXED:
        // keep heading pointing in the direction held in fixed_yaw
        // caller should set the fixed_yaw
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        _look_ahead_yaw = copter.ahrs.yaw_sensor;
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // initial_armed_bearing will be set during arming so no init required
        break;

    case AUTO_YAW_RATE:
        // initialise target yaw rate to zero
        _yaw_rate_cds = 0.0f;
        break;

    case AUTO_YAW_CIRCLE:
        // no initialisation required
        break;
    }
}

// set_fixed_yaw - sets the yaw look at heading for auto mode
void Mode::AutoYaw::set_fixed_yaw(float angle_deg, float turn_rate_ds, int8_t direction, bool relative_angle)
{
    _last_update_ms = millis();

    _yaw_angle_cd = copter.attitude_control->get_att_target_euler_cd().z;
    _yaw_rate_cds = 0.0;

    // calculate final angle as relative to vehicle heading or absolute
    if (relative_angle) {
        _fixed_yaw_offset_cd = angle_deg * 100.0 * (direction >= 0 ? 1.0 : -1.0);
    } else {
        // absolute angle
        _fixed_yaw_offset_cd = wrap_180_cd(angle_deg * 100.0 - _yaw_angle_cd);
        if ( direction < 0 && is_positive(_fixed_yaw_offset_cd) ) {
            _fixed_yaw_offset_cd -= 36000.0;
        } else if ( direction > 0 && is_negative(_fixed_yaw_offset_cd) ) {
            _fixed_yaw_offset_cd += 36000.0;
        }
    }

    // get turn speed
    if (!is_positive(turn_rate_ds)) {
        // default to default slew rate
        _fixed_yaw_slewrate_cds = copter.attitude_control->get_slew_yaw_cds();
    } else {
        _fixed_yaw_slewrate_cds = MIN(copter.attitude_control->get_slew_yaw_cds(), turn_rate_ds * 100.0);
    }

    // set yaw mode
    set_mode(AUTO_YAW_FIXED);
}

// set_fixed_yaw - sets the yaw look at heading for auto mode
void Mode::AutoYaw::set_yaw_angle_rate(float yaw_angle_d, float yaw_rate_ds)
{
    _last_update_ms = millis();

    _yaw_angle_cd = yaw_angle_d * 100.0;
    _yaw_rate_cds = yaw_rate_ds * 100.0;

    // set yaw mode
    set_mode(AUTO_YAW_ANGLE_RATE);
}

// set_roi - sets the yaw to look at roi for auto mode
void Mode::AutoYaw::set_roi(const Location &roi_location)
{
    // if location is zero lat, lon and altitude turn off ROI
    if (roi_location.alt == 0 && roi_location.lat == 0 && roi_location.lng == 0) {
        // set auto yaw mode back to default assuming the active command is a waypoint command.  A more sophisticated method is required to ensure we return to the proper yaw control for the active command
        auto_yaw.set_mode_to_default(false);
#if HAL_MOUNT_ENABLED
        // switch off the camera tracking if enabled
        if (copter.camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
            copter.camera_mount.set_mode_to_default();
        }
#endif  // HAL_MOUNT_ENABLED
    } else {
#if HAL_MOUNT_ENABLED
        // check if mount type requires us to rotate the quad
        if (!copter.camera_mount.has_pan_control()) {
            if (roi_location.get_vector_from_origin_NEU(roi)) {
                auto_yaw.set_mode(AUTO_YAW_ROI);
            }
        }
        // send the command to the camera mount
        copter.camera_mount.set_roi_target(roi_location);

        // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
        //      0: do nothing
        //      1: point at next waypoint
        //      2: point at a waypoint taken from WP# parameter (2nd parameter?)
        //      3: point at a location given by alt, lon, lat parameters
        //      4: point at a target given a target id (can't be implemented)
#else
        // if we have no camera mount aim the quad at the location
        if (roi_location.get_vector_from_origin_NEU(roi)) {
            auto_yaw.set_mode(AUTO_YAW_ROI);
        }
#endif  // HAL_MOUNT_ENABLED
    }
}

// set auto yaw rate in centi-degrees per second
void Mode::AutoYaw::set_rate(float turn_rate_cds)
{
    set_mode(AUTO_YAW_RATE);
    _yaw_rate_cds = turn_rate_cds;
}

// yaw - returns target heading depending upon auto_yaw.mode()
float Mode::AutoYaw::yaw()
{
    switch (_mode) {

    case AUTO_YAW_ROI:
        // point towards a location held in roi
        return roi_yaw();

    case AUTO_YAW_FIXED: {
        // keep heading pointing in the direction held in fixed_yaw
        // with no pilot input allowed
        const uint32_t now_ms = millis();
        float dt = (now_ms - _last_update_ms) * 0.001;
        _last_update_ms = now_ms;
        float yaw_angle_step = constrain_float(_fixed_yaw_offset_cd, - dt * _fixed_yaw_slewrate_cds, dt * _fixed_yaw_slewrate_cds);
        _fixed_yaw_offset_cd -= yaw_angle_step;
        _yaw_angle_cd += yaw_angle_step;
        return _yaw_angle_cd;
    }

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        return look_ahead_yaw();

    case AUTO_YAW_RESETTOARMEDYAW:
        // changes yaw to be same as when quad was armed
        return copter.initial_armed_bearing;

    case AUTO_YAW_CIRCLE:
#if MODE_CIRCLE_ENABLED
        if (copter.circle_nav->is_active()) {
            return copter.circle_nav->get_yaw();
        }
#endif
        // return the current attitude target
        return wrap_360_cd(copter.attitude_control->get_att_target_euler_cd().z);

    case AUTO_YAW_ANGLE_RATE:{
        const uint32_t now_ms = millis();
        float dt = (now_ms - _last_update_ms) * 0.001;
        _last_update_ms = now_ms;
        _yaw_angle_cd += _yaw_rate_cds * dt;
        return _yaw_angle_cd;
    }

    case AUTO_YAW_LOOK_AT_NEXT_WP:
    default:
        // point towards next waypoint.
        // we don't use wp_bearing because we don't want the copter to turn too much during flight
        return copter.pos_control->get_yaw_cd();
    }
}

// returns yaw rate normally set by SET_POSITION_TARGET mavlink
// messages (positive is clockwise, negative is counter clockwise)
float Mode::AutoYaw::rate_cds() const
{
    switch (_mode) {

    case AUTO_YAW_HOLD:
    case AUTO_YAW_ROI:
    case AUTO_YAW_FIXED:
    case AUTO_YAW_LOOK_AHEAD:
    case AUTO_YAW_RESETTOARMEDYAW:
    case AUTO_YAW_CIRCLE:
        return 0.0f;

    case AUTO_YAW_ANGLE_RATE:
    case AUTO_YAW_RATE:
        return _yaw_rate_cds;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
        return copter.pos_control->get_yaw_rate_cds();
    }

    // return zero turn rate (this should never happen)
    return 0.0f;
}
