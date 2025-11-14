#include "Copter.h"

Mode::AutoYaw Mode::auto_yaw;

// roi_yaw_rad - returns heading towards location held in roi_ned_m
float Mode::AutoYaw::roi_yaw_rad() const
{
    Vector2f pos_ne_m;
    if (AP::ahrs().get_relative_position_NE_origin_float(pos_ne_m)){
        return get_bearing_rad(pos_ne_m, roi_ned_m.xy());
    }
    return copter.attitude_control->get_att_target_euler_rad().z;
}

// Returns the yaw angle (in radians) representing the direction of horizontal motion.
float Mode::AutoYaw::look_ahead_yaw_rad()
{
    // Commanded Yaw to automatically look ahead.
    Vector3f vel_ned_ms;
    if (copter.position_ok() && AP::ahrs().get_velocity_NED(vel_ned_ms)) {
        const float speed_ms_sq = vel_ned_ms.xy().length_squared();
        if (speed_ms_sq > (YAW_LOOK_AHEAD_MIN_SPEED_MS * YAW_LOOK_AHEAD_MIN_SPEED_MS)) {
            _look_ahead_yaw_rad = atan2f(vel_ned_ms.y,vel_ned_ms.x);
        }
    }
    return _look_ahead_yaw_rad;
}

void Mode::AutoYaw::set_mode_to_default(bool rtl)
{
    set_mode(default_mode(rtl));
}

// default_mode - returns auto_yaw.mode() based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
Mode::AutoYaw::Mode Mode::AutoYaw::default_mode(bool rtl) const
{
    switch (copter.g.wp_yaw_behavior) {

    case WP_YAW_BEHAVIOR_NONE:
        return Mode::HOLD;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
        if (rtl) {
            return Mode::HOLD;
        } else {
            return Mode::LOOK_AT_NEXT_WP;
        }

    case WP_YAW_BEHAVIOR_LOOK_AHEAD:
        return Mode::LOOK_AHEAD;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
    default:
        return Mode::LOOK_AT_NEXT_WP;
    }
}

// set_mode - sets the yaw mode for auto
void Mode::AutoYaw::set_mode(Mode yaw_mode)
{
    // return immediately if no change
    if (_mode == yaw_mode) {
        return;
    }
    _last_mode = _mode;
    _mode = yaw_mode;

    // perform initialisation
    switch (_mode) {

    case Mode::HOLD:
        break;

    case Mode::LOOK_AT_NEXT_WP:
        // wpnav will initialise heading when wpnav's set_destination method is called
        break;

    case Mode::ROI:
        // look ahead until we know otherwise
        break;

    case Mode::FIXED:
        // keep heading pointing in the direction held in fixed_yaw
        // caller should set the fixed_yaw
        break;

    case Mode::LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        _look_ahead_yaw_rad = copter.ahrs.get_yaw_rad();
        break;

    case Mode::RESET_TO_ARMED_YAW:
        // initial_armed_bearing_rad will be set during arming so no init required
        break;

    case Mode::ANGLE_RATE:
        break;

    case Mode::RATE:
        // initialise target yaw rate to zero
        _yaw_rate_rads = 0.0;
        break;

    case Mode::CIRCLE:
    case Mode::PILOT_RATE:
    case Mode::WEATHERVANE:
        // no initialisation required
        break;
    }
}

// set_fixed_yaw_rad - sets the yaw look at heading for auto mode
void Mode::AutoYaw::set_fixed_yaw_rad(float yaw_rad, float yaw_rate_rads, int8_t direction, bool relative_angle)
{
    _last_update_ms = millis();
    const float angle_rad = yaw_rad;

    // calculate final angle as relative to vehicle heading or absolute
    if (relative_angle) {
        if (_mode == Mode::HOLD) {
            _yaw_angle_rad = copter.ahrs.get_yaw_rad();
        }
        _fixed_yaw_offset_rad = angle_rad * (direction >= 0 ? 1.0 : -1.0);
    } else {
        // absolute angle
        _fixed_yaw_offset_rad = wrap_PI(angle_rad - _yaw_angle_rad);
        if (direction < 0 && is_positive(_fixed_yaw_offset_rad)) {
            _fixed_yaw_offset_rad -= M_2PI;
        } else if (direction > 0 && is_negative(_fixed_yaw_offset_rad)) {
            _fixed_yaw_offset_rad += M_2PI;
        }
    }

    // get turn speed
    if (!is_positive(yaw_rate_rads)) {
        // default to default slew rate
        _fixed_yaw_slewrate_rads = copter.attitude_control->get_slew_yaw_max_rads();
    } else {
        _fixed_yaw_slewrate_rads = MIN(copter.attitude_control->get_slew_yaw_max_rads(), yaw_rate_rads);
    }

    // set yaw mode
    set_mode(Mode::FIXED);
}

// set_fixed_yaw_rad - sets the yaw look at heading for auto mode
void Mode::AutoYaw::set_yaw_angle_and_rate_rad(float yaw_angle_rad, float yaw_rate_rads)
{
    _last_update_ms = millis();

    _yaw_angle_rad = yaw_angle_rad;
    _yaw_rate_rads = yaw_rate_rads;

    // set yaw mode
    set_mode(Mode::ANGLE_RATE);
}

// set_yaw_angle_offset_deg - sets the yaw look at heading for auto mode, as an offset from the current yaw angle
void Mode::AutoYaw::set_yaw_angle_offset_deg(const float yaw_angle_offset_deg)
{
    _last_update_ms = millis();

    _yaw_angle_rad = wrap_2PI(_yaw_angle_rad + radians(yaw_angle_offset_deg));
    _yaw_rate_rads = 0.0f;

    // set yaw mode
    set_mode(Mode::ANGLE_RATE);
}

// set_roi - sets the yaw to look at roi_ned_m for auto mode
void Mode::AutoYaw::set_roi(const Location &roi_location)
{
    // if location is zero lat, lon and altitude turn off ROI
    if (!roi_location.initialised()) {
        // set auto yaw mode back to default assuming the active command is a waypoint command.  A more sophisticated method is required to ensure we return to the proper yaw control for the active command
        auto_yaw.set_mode_to_default(false);
#if HAL_MOUNT_ENABLED
        // switch off the camera tracking if enabled
        copter.camera_mount.clear_roi_target();
#endif  // HAL_MOUNT_ENABLED
    } else {
#if HAL_MOUNT_ENABLED
        // check if mount type requires us to rotate the quad
        if (!copter.camera_mount.has_pan_control()) {
            if (roi_location.get_vector_from_origin_NED_m(roi_ned_m)) {
                auto_yaw.set_mode(Mode::ROI);
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
        if (roi_location.get_vector_from_origin_NED_m(roi_ned_m)) {
            auto_yaw.set_mode(Mode::ROI);
        }
#endif  // HAL_MOUNT_ENABLED
    }
}

// set auto yaw rate in radians per second
void Mode::AutoYaw::set_rate_rad(float turn_rate_rads)
{
    set_mode(Mode::RATE);
    _yaw_rate_rads = turn_rate_rads;
}

// return true if fixed yaw target has been reached
bool Mode::AutoYaw::reached_fixed_yaw_target()
{
    if (mode() != Mode::FIXED) {
        // should not happen, not in the right mode
        return true;
    }

    if (!is_zero(_fixed_yaw_offset_rad)) {
        // still slewing yaw target
        return false;
    }

    // Within 2 deg of target
    return (fabsf(wrap_PI(_yaw_angle_rad - copter.ahrs.get_yaw_rad())) <= radians(2));
}

// yaw_rad - returns target heading depending upon auto_yaw.mode()
float Mode::AutoYaw::yaw_rad()
{
    switch (_mode) {

    case Mode::ROI:
        // point towards a location held in roi_ned_m
        _yaw_angle_rad = roi_yaw_rad();
        break;

    case Mode::FIXED: {
        // keep heading pointing in the direction held in fixed_yaw
        // with no pilot input allowed
        const uint32_t now_ms = millis();
        float dt = (now_ms - _last_update_ms) * 0.001;
        _last_update_ms = now_ms;
        float yaw_angle_step_rad = constrain_float(_fixed_yaw_offset_rad, - dt * _fixed_yaw_slewrate_rads, dt * _fixed_yaw_slewrate_rads);
        _fixed_yaw_offset_rad -= yaw_angle_step_rad;
        _yaw_angle_rad += yaw_angle_step_rad;
        break;
    }

    case Mode::LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        _yaw_angle_rad = look_ahead_yaw_rad();
        break;

    case Mode::RESET_TO_ARMED_YAW:
        // changes yaw to be same as when quad was armed
        _yaw_angle_rad = copter.initial_armed_bearing_rad;
        break;

    case Mode::CIRCLE:
#if MODE_CIRCLE_ENABLED
        if (copter.circle_nav->is_active()) {
            _yaw_angle_rad = copter.circle_nav->get_yaw_rad();
        }
#endif
        break;

    case Mode::ANGLE_RATE:{
        const uint32_t now_ms = millis();
        float dt = (now_ms - _last_update_ms) * 0.001;
        _last_update_ms = now_ms;
        _yaw_angle_rad += _yaw_rate_rads * dt;
        break;
    }

    case Mode::RATE:
    case Mode::WEATHERVANE:
    case Mode::PILOT_RATE:
        _yaw_angle_rad = copter.attitude_control->get_att_target_euler_rad().z;
        break;

    case Mode::LOOK_AT_NEXT_WP:
    default:
        // point towards next waypoint.
        // we don't use wp_bearing_deg because we don't want the copter to turn too much during flight
        _yaw_angle_rad = copter.pos_control->get_yaw_rad();
    break;
    }
    return _yaw_angle_rad;
}

// returns yaw rate normally set by SET_POSITION_TARGET mavlink
// messages (positive is clockwise, negative is counter clockwise)
float Mode::AutoYaw::rate_rads()
{
    switch (_mode) {

    case Mode::HOLD:
    case Mode::ROI:
    case Mode::FIXED:
    case Mode::LOOK_AHEAD:
    case Mode::RESET_TO_ARMED_YAW:
    case Mode::CIRCLE:
        _yaw_rate_rads = 0.0f;
        break;

    case Mode::LOOK_AT_NEXT_WP:
        _yaw_rate_rads = copter.pos_control->get_yaw_rate_rads();
        break;

    case Mode::PILOT_RATE:
        _yaw_rate_rads = _pilot_yaw_rate_rads;
        break;

    case Mode::ANGLE_RATE:
    case Mode::RATE:
    case Mode::WEATHERVANE:
        break;
    }

    // return zero turn rate (this should never happen)
    return _yaw_rate_rads;
}

AC_AttitudeControl::HeadingCommand Mode::AutoYaw::get_heading()
{
    // process pilot's yaw input
    _pilot_yaw_rate_rads = 0.0;
    if (rc().has_valid_input() && copter.flightmode->use_pilot_yaw()) {
        // get pilot's desired yaw rate
        _pilot_yaw_rate_rads = copter.flightmode->get_pilot_desired_yaw_rate_rads();
        if (!is_zero(_pilot_yaw_rate_rads)) {
            auto_yaw.set_mode(AutoYaw::Mode::PILOT_RATE);
        }
    } else if (auto_yaw.mode() == AutoYaw::Mode::PILOT_RATE) {
        // RC failsafe, or disabled make sure not in pilot control
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

#if WEATHERVANE_ENABLED
    update_weathervane(_pilot_yaw_rate_rads);
#endif

    AC_AttitudeControl::HeadingCommand heading;
    heading.yaw_angle_rad = auto_yaw.yaw_rad();
    heading.yaw_rate_rads = auto_yaw.rate_rads();

    switch (auto_yaw.mode()) {
        case Mode::HOLD:
        case Mode::RATE:
        case Mode::PILOT_RATE:
        case Mode::WEATHERVANE:
            heading.heading_mode = AC_AttitudeControl::HeadingMode::Rate_Only;
            break;
        case Mode::LOOK_AT_NEXT_WP:
        case Mode::ROI:
        case Mode::FIXED:
        case Mode::LOOK_AHEAD:
        case Mode::RESET_TO_ARMED_YAW:
        case Mode::ANGLE_RATE:
        case Mode::CIRCLE:
            heading.heading_mode = AC_AttitudeControl::HeadingMode::Angle_And_Rate;
            break;
    }

    return heading;
}

// handle the interface to the weathervane library
// pilot_yaw can be an angle or a rate or rcin from yaw channel. It just needs to represent a pilot's request to yaw the vehicle to enable pilot overrides.
#if WEATHERVANE_ENABLED
void Mode::AutoYaw::update_weathervane(const float pilot_yaw_rads)
{
    if (!copter.flightmode->allows_weathervaning()) {
        return;
    }

    float yaw_rate_cds;
    if (copter.g2.weathervane.get_yaw_out(yaw_rate_cds, rad_to_cd(pilot_yaw_rads), copter.flightmode->get_alt_above_ground_m(),
                                                                       copter.pos_control->get_roll_cd()-copter.attitude_control->get_roll_trim_cd(),
                                                                       copter.pos_control->get_pitch_cd(),
                                                                       copter.flightmode->is_taking_off(),
                                                                       copter.flightmode->is_landing())) {
        set_mode(Mode::WEATHERVANE);
        _yaw_rate_rads = cd_to_rad(yaw_rate_cds);
        return;
    }

    // if the weathervane controller has previously been activated we need to ensure we return control back to what was previously set
    if (mode() == Mode::WEATHERVANE) {
        _yaw_rate_rads = 0.0;
        if (_last_mode == Mode::HOLD) {
            set_mode_to_default(false);
        } else {
            set_mode(_last_mode);
        }
    }
}
#endif // WEATHERVANE_ENABLED
