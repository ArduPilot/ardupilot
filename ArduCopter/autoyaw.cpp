#include "Copter.h"

Copter::Mode::AutoYaw Copter::Mode::auto_yaw;

// roi_yaw - returns heading towards location held in roi
float Copter::Mode::AutoYaw::roi_yaw()
{
    roi_yaw_counter++;
    if (roi_yaw_counter >= 4) {
        roi_yaw_counter = 0;
        _roi_yaw = get_bearing_cd(copter.inertial_nav.get_position(), roi);
    }

    return _roi_yaw;
}

float Copter::Mode::AutoYaw::look_ahead_yaw()
{
    const Vector3f& vel = copter.inertial_nav.get_velocity();
    float speed = norm(vel.x,vel.y);
    // Commanded Yaw to automatically look ahead.
    if (copter.position_ok() && (speed > YAW_LOOK_AHEAD_MIN_SPEED)) {
        _look_ahead_yaw = degrees(atan2f(vel.y,vel.x))*100.0f;
    }
    return _look_ahead_yaw;
}

void Copter::Mode::AutoYaw::set_mode_to_default(bool rtl)
{
    set_mode(default_mode(rtl));
}

// default_mode - returns auto_yaw.mode() based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
autopilot_yaw_mode Copter::Mode::AutoYaw::default_mode(bool rtl) const
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
void Copter::Mode::AutoYaw::set_mode(autopilot_yaw_mode yaw_mode)
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
        _roi_yaw = copter.ahrs.yaw_sensor;
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
        _rate_cds = 0.0f;
        break;
    }
}

// set_fixed_yaw - sets the yaw look at heading for auto mode
void Copter::Mode::AutoYaw::set_fixed_yaw(float angle_deg, float turn_rate_dps, int8_t direction, bool relative_angle)
{
    const int32_t curr_yaw_target = copter.attitude_control->get_att_target_euler_cd().z;

    // calculate final angle as relative to vehicle heading or absolute
    if (!relative_angle) {
        // absolute angle
        _fixed_yaw = wrap_360_cd(angle_deg * 100);
    } else {
        // relative angle
        if (direction < 0) {
            angle_deg = -angle_deg;
        }
        _fixed_yaw = wrap_360_cd((angle_deg * 100) + curr_yaw_target);
    }

    // get turn speed
    if (is_zero(turn_rate_dps)) {
        // default to regular auto slew rate
        _fixed_yaw_slewrate = AUTO_YAW_SLEW_RATE;
    } else {
        const int32_t turn_rate = (wrap_180_cd(_fixed_yaw - curr_yaw_target) / 100) / turn_rate_dps;
        _fixed_yaw_slewrate = constrain_int32(turn_rate, 1, 360);    // deg / sec
    }

    // set yaw mode
    set_mode(AUTO_YAW_FIXED);

    // TO-DO: restore support for clockwise and counter clockwise rotation held in cmd.content.yaw.direction.  1 = clockwise, -1 = counterclockwise
}

// set_roi - sets the yaw to look at roi for auto mode
void Copter::Mode::AutoYaw::set_roi(const Location &roi_location)
{
    // if location is zero lat, lon and altitude turn off ROI
    if (roi_location.alt == 0 && roi_location.lat == 0 && roi_location.lng == 0) {
        // set auto yaw mode back to default assuming the active command is a waypoint command.  A more sophisticated method is required to ensure we return to the proper yaw control for the active command
        auto_yaw.set_mode_to_default(false);
#if MOUNT == ENABLED
        // switch off the camera tracking if enabled
        if (copter.camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
            copter.camera_mount.set_mode_to_default();
        }
#endif  // MOUNT == ENABLED
    } else {
#if MOUNT == ENABLED
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
#endif  // MOUNT == ENABLED
    }
}

// set auto yaw rate in centi-degrees per second
void Copter::Mode::AutoYaw::set_rate(float turn_rate_cds)
{
    set_mode(AUTO_YAW_RATE);
    _rate_cds = turn_rate_cds;
}

// yaw - returns target heading depending upon auto_yaw.mode()
float Copter::Mode::AutoYaw::yaw()
{
    switch (_mode) {

    case AUTO_YAW_ROI:
        // point towards a location held in roi
        return roi_yaw();

    case AUTO_YAW_FIXED:
        // keep heading pointing in the direction held in fixed_yaw
        // with no pilot input allowed
        return _fixed_yaw;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        return look_ahead_yaw();

    case AUTO_YAW_RESETTOARMEDYAW:
        // changes yaw to be same as when quad was armed
        return copter.initial_armed_bearing;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
    default:
        // point towards next waypoint.
        // we don't use wp_bearing because we don't want the copter to turn too much during flight
        return copter.wp_nav->get_yaw();
    }
}

// returns yaw rate normally set by SET_POSITION_TARGET mavlink
// messages (positive is clockwise, negative is counter clockwise)
float Copter::Mode::AutoYaw::rate_cds() const
{
    if (_mode == AUTO_YAW_RATE) {
        return _rate_cds;
    }

    // return zero turn rate (this should never happen)
    return 0.0f;
}
