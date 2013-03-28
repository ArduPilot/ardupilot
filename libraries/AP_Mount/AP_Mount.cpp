// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Mount.h>

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

#if defined( __AVR_ATmega1280__ )
 # define MNT_JSTICK_SPD_OPTION DISABLED // Allow RC joystick to control the speed of the mount movements instead of the position of the mount
 # define MNT_RETRACT_OPTION    DISABLED // Use a servo to retract the mount inside the fuselage (i.e. for landings)
 # define MNT_GPSPOINT_OPTION    ENABLED // Point the mount to a GPS point defined via a mouse click in the Mission Planner GUI
 # define MNT_STABILIZE_OPTION  DISABLED // stabilize camera using frame attitude information
 # define MNT_MOUNT2_OPTION     DISABLED // second mount, can for example be used to keep an antenna pointed at the home position
#else
 # define MNT_JSTICK_SPD_OPTION ENABLED // uses  844 bytes of memory
 # define MNT_RETRACT_OPTION    ENABLED // uses  244 bytes of memory
 # define MNT_GPSPOINT_OPTION   ENABLED // uses  580 bytes of memory
 # define MNT_STABILIZE_OPTION  ENABLED // uses 2424 bytes of memory
 # define MNT_MOUNT2_OPTION     ENABLED // uses   58 bytes of memory (must also be enabled in APM_Config.h)
#endif

const AP_Param::GroupInfo AP_Mount::var_info[] PROGMEM = {
    // @Param: MODE
    // @DisplayName: Mount operation mode
    // @Description: Camera or antenna mount operation mode
    // @Values: 0:retract,1:neutral,2:MavLink_targeting,3:RC_targeting,4:GPS_point
    // @User: Standard
    AP_GROUPINFO("MODE",       0, AP_Mount, _mount_mode, MAV_MOUNT_MODE_RETRACT), // see MAV_MOUNT_MODE at ardupilotmega.h

#if MNT_RETRACT_OPTION == ENABLED
    // @Param: RETRACT_X
    // @DisplayName: Mount roll angle when in retracted position
    // @Description: Mount roll angle when in retracted position
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard

    // @Param: RETRACT_Y
    // @DisplayName: Mount tilt/pitch angle when in retracted position
    // @Description: Mount tilt/pitch angle when in retracted position
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard

    // @Param: RETRACT_Z
    // @DisplayName: Mount yaw/pan angle when in retracted position
    // @Description: Mount yaw/pan angle when in retracted position
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RETRACT",    1, AP_Mount, _retract_angles, 0),
#endif

    // @Param: NEUTRAL_X
    // @DisplayName: Mount roll angle when in neutral position
    // @Description: Mount roll angle when in neutral position
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard

    // @Param: NEUTRAL_Y
    // @DisplayName: Mount tilt/pitch angle when in neutral position
    // @Description: Mount tilt/pitch angle when in neutral position
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard

    // @Param: NEUTRAL_Z
    // @DisplayName: Mount pan/yaw angle when in neutral position
    // @Description: Mount pan/yaw angle when in neutral position
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("NEUTRAL",    2, AP_Mount, _neutral_angles, 0),

    // @Param: CONTROL_X
    // @DisplayName: Mount roll angle command from groundstation
    // @Description: Mount roll angle when in MavLink or RC control operation mode
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1

    // @Param: CONTROL_Y
    // @DisplayName: Mount tilt/pitch angle command from groundstation
    // @Description: Mount tilt/pitch angle when in MavLink or RC control operation mode
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1

    // @Param: CONTROL_Z
    // @DisplayName: Mount pan/yaw angle command from groundstation
    // @Description: Mount pan/yaw angle when in MavLink or RC control operation mode
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    AP_GROUPINFO("CONTROL",    3, AP_Mount, _control_angles, 0),

#if MNT_STABILIZE_OPTION == ENABLED
    // @Param: STAB_ROLL
    // @DisplayName: Stabilize mount's roll angle
    // @Description:enable roll stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("STAB_ROLL",  4, AP_Mount, _stab_roll, 0),

    // @Param: STAB_TILT
    // @DisplayName: Stabilize mount's pitch/tilt angle
    // @Description: enable tilt/pitch stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("STAB_TILT", 5, AP_Mount, _stab_tilt,  0),

    // @Param: STAB_PAN
    // @DisplayName: Stabilize mount pan/yaw angle
    // @Description: enable pan/yaw stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("STAB_PAN",   6, AP_Mount, _stab_pan,  0),
#endif

    // @Param: RC_IN_ROLL
    // @DisplayName: roll RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control roll movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8
    // @User: Standard
    AP_GROUPINFO("RC_IN_ROLL",  7, AP_Mount, _roll_rc_in, 0),

    // @Param: ANGMIN_ROL
    // @DisplayName: Minimum roll angle
    // @Description: Minimum physical roll angular position of mount.
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_ROL", 8, AP_Mount, _roll_angle_min, -4500),

    // @Param: ANGMAX_ROL
    // @DisplayName: Maximum roll angle
    // @Description: Maximum physical roll angular position of the mount
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_ROL", 9, AP_Mount, _roll_angle_max, 4500),

    // @Param: RC_IN_TILT
    // @DisplayName: tilt (pitch) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8
    // @User: Standard
    AP_GROUPINFO("RC_IN_TILT",  10, AP_Mount, _tilt_rc_in,    0),

    // @Param: ANGMIN_TIL
    // @DisplayName: Minimum tilt angle
    // @Description: Minimum physical tilt (pitch) angular position of mount.
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_TIL", 11, AP_Mount, _tilt_angle_min, -4500),

    // @Param: ANGMAX_TIL
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum physical tilt (pitch) angular position of the mount
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_TIL", 12, AP_Mount, _tilt_angle_max, 4500),

    // @Param: RC_IN_PAN
    // @DisplayName: pan (yaw) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control pan (yaw) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8
    // @User: Standard
    AP_GROUPINFO("RC_IN_PAN",  13, AP_Mount, _pan_rc_in,       0),

    // @Param: ANGMIN_PAN
    // @DisplayName: Minimum pan angle
    // @Description: Minimum physical pan (yaw) angular position of mount.
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_PAN",  14, AP_Mount, _pan_angle_min,  -4500),

    // @Param: ANGMAX_PAN
    // @DisplayName: Maximum pan angle
    // @Description: Maximum physical pan (yaw) angular position of the mount
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_PAN",  15, AP_Mount, _pan_angle_max,  4500),

#if MNT_JSTICK_SPD_OPTION == ENABLED
    // @Param: JSTICK_SPD
    // @DisplayName: mount joystick speed
    // @Description: 0 for position control, small for low speeds, 100 for max speed. A good general value is 10 which gives a movement speed of 3 degrees per second.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("JSTICK_SPD",  16, AP_Mount, _joystick_speed, 0),
#endif

    AP_GROUPEND
};

extern RC_Channel* rc_ch[8];

AP_Mount::AP_Mount(const struct Location *current_loc, GPS *&gps, AP_AHRS *ahrs, uint8_t id) :
    _gps(gps)
{
	AP_Param::setup_object_defaults(this, var_info);
    _ahrs = ahrs;
    _current_loc = current_loc;

    // default to zero angles
    _retract_angles = Vector3f(0,0,0);
    _neutral_angles = Vector3f(0,0,0);
    _control_angles = Vector3f(0,0,0);

    // default unknown mount type
    _mount_type = k_unknown;

#if MNT_MOUNT2_OPTION == ENABLED
    if (id == 0) {
#endif
        _roll_idx = RC_Channel_aux::k_mount_roll;
        _tilt_idx = RC_Channel_aux::k_mount_tilt;
        _pan_idx  = RC_Channel_aux::k_mount_pan;
        _open_idx = RC_Channel_aux::k_mount_open;
#if MNT_MOUNT2_OPTION == ENABLED
    } else {
        _roll_idx = RC_Channel_aux::k_mount2_roll;
        _tilt_idx = RC_Channel_aux::k_mount2_tilt;
        _pan_idx  = RC_Channel_aux::k_mount2_pan;
        _open_idx = RC_Channel_aux::k_mount2_open;
    }
#endif
}

/// Auto-detect the mount gimbal type depending on the functions assigned to the servos
void
AP_Mount::update_mount_type()
{
	bool have_roll, have_tilt, have_pan;
	have_roll = RC_Channel_aux::function_assigned(RC_Channel_aux::k_mount_roll) ||
		RC_Channel_aux::function_assigned(RC_Channel_aux::k_mount2_roll);
	have_tilt = RC_Channel_aux::function_assigned(RC_Channel_aux::k_mount_tilt) ||
		RC_Channel_aux::function_assigned(RC_Channel_aux::k_mount2_tilt);
	have_pan = RC_Channel_aux::function_assigned(RC_Channel_aux::k_mount_pan) ||
		RC_Channel_aux::function_assigned(RC_Channel_aux::k_mount2_pan);
    if (have_pan && have_tilt && !have_roll) {
        _mount_type = k_pan_tilt;
    }
    if (!have_pan && have_tilt && have_roll) {
        _mount_type = k_tilt_roll;
    }
    if (have_pan && have_tilt && have_roll) {
        _mount_type = k_pan_tilt_roll;
    }
}

/// sets the servo angles for retraction, note angles are in degrees
void AP_Mount::set_retract_angles(float roll, float tilt, float pan)
{
    _retract_angles = Vector3f(roll, tilt, pan);
}

//sets the servo angles for neutral, note angles are in degrees
void AP_Mount::set_neutral_angles(float roll, float tilt, float pan)
{
    _neutral_angles = Vector3f(roll, tilt, pan);
}

/// sets the servo angles for MAVLink, note angles are in degrees
void AP_Mount::set_control_angles(float roll, float tilt, float pan)
{
    _control_angles = Vector3f(roll, tilt, pan);
}

/// used to tell the mount to track GPS location
void AP_Mount::set_GPS_target_location(Location targetGPSLocation)
{
    _target_GPS_location=targetGPSLocation;
}

/// This one should be called periodically
void AP_Mount::update_mount_position()
{
#if MNT_RETRACT_OPTION == ENABLED
    static bool mount_open = 0;     // 0 is closed
#endif

    switch((enum MAV_MOUNT_MODE)_mount_mode.get())
    {
#if MNT_RETRACT_OPTION == ENABLED
    // move mount to a "retracted position" or to a position where a fourth servo can retract the entire mount into the fuselage
    case MAV_MOUNT_MODE_RETRACT:
    {
        Vector3f vec = _retract_angles.get();
        _roll_angle  = vec.x;
        _tilt_angle  = vec.y;
        _pan_angle   = vec.z;
        break;
    }
#endif

    // move mount to a neutral position, typically pointing forward
    case MAV_MOUNT_MODE_NEUTRAL:
    {
        Vector3f vec = _neutral_angles.get();
        _roll_angle  = vec.x;
        _tilt_angle  = vec.y;
        _pan_angle   = vec.z;
        break;
    }

    // point to the angles given by a mavlink message
    case MAV_MOUNT_MODE_MAVLINK_TARGETING:
    {
        Vector3f vec = _control_angles.get();
        _roll_control_angle  = radians(vec.x);
        _tilt_control_angle  = radians(vec.y);
        _pan_control_angle   = radians(vec.z);
        stabilize();
        break;
    }

    // RC radio manual angle control, but with stabilization from the AHRS
    case MAV_MOUNT_MODE_RC_TARGETING:
    {
#if MNT_JSTICK_SPD_OPTION == ENABLED
        if (_joystick_speed) {                  // for spring loaded joysticks
            // allow pilot speed position input to come directly from an RC_Channel
            if (_roll_rc_in && (rc_ch[_roll_rc_in-1])) {
                _roll_control_angle += rc_ch[_roll_rc_in-1]->norm_input() * 0.0001f * _joystick_speed;
                if (_roll_control_angle < radians(_roll_angle_min*0.01f)) _roll_control_angle = radians(_roll_angle_min*0.01f);
                if (_roll_control_angle > radians(_roll_angle_max*0.01f)) _roll_control_angle = radians(_roll_angle_max*0.01f);
            }
            if (_tilt_rc_in && (rc_ch[_tilt_rc_in-1])) {
                _tilt_control_angle += rc_ch[_tilt_rc_in-1]->norm_input() * 0.0001f * _joystick_speed;
                if (_tilt_control_angle < radians(_tilt_angle_min*0.01f)) _tilt_control_angle = radians(_tilt_angle_min*0.01f);
                if (_tilt_control_angle > radians(_tilt_angle_max*0.01f)) _tilt_control_angle = radians(_tilt_angle_max*0.01f);
            }
            if (_pan_rc_in && (rc_ch[_pan_rc_in-1])) {
                _pan_control_angle += rc_ch[_pan_rc_in-1]->norm_input() * 0.0001f * _joystick_speed;
                if (_pan_control_angle < radians(_pan_angle_min*0.01f)) _pan_control_angle = radians(_pan_angle_min*0.01f);
                if (_pan_control_angle > radians(_pan_angle_max*0.01f)) _pan_control_angle = radians(_pan_angle_max*0.01f);
            }
        } else {
#endif
            // allow pilot position input to come directly from an RC_Channel
            if (_roll_rc_in && (rc_ch[_roll_rc_in-1])) {
                _roll_control_angle = angle_input_rad(rc_ch[_roll_rc_in-1], _roll_angle_min, _roll_angle_max);
            }
            if (_tilt_rc_in && (rc_ch[_tilt_rc_in-1])) {
                _tilt_control_angle = angle_input_rad(rc_ch[_tilt_rc_in-1], _tilt_angle_min, _tilt_angle_max);
            }
            if (_pan_rc_in && (rc_ch[_pan_rc_in-1])) {
                _pan_control_angle = angle_input_rad(rc_ch[_pan_rc_in-1], _pan_angle_min, _pan_angle_max);
            }
#if MNT_JSTICK_SPD_OPTION == ENABLED
        }
#endif
        stabilize();
        break;
    }

#if MNT_GPSPOINT_OPTION == ENABLED
    // point mount to a GPS point given by the mission planner
    case MAV_MOUNT_MODE_GPS_POINT:
    {
        if(_gps->fix) {
            calc_GPS_target_angle(&_target_GPS_location);
            stabilize();
        }
        break;
    }
#endif

    default:
        //do nothing
        break;
    }

#if MNT_RETRACT_OPTION == ENABLED
    // move mount to a "retracted position" into the fuselage with a fourth servo
	bool mount_open_new = (enum MAV_MOUNT_MODE)_mount_mode.get()==MAV_MOUNT_MODE_RETRACT ? 0 : 1;
	if (mount_open != mount_open_new) {
		mount_open = mount_open_new;
		move_servo(_open_idx, mount_open_new, 0, 1);
    }
#endif

    // write the results to the servos
    move_servo(_roll_idx, _roll_angle*10, _roll_angle_min*0.1f, _roll_angle_max*0.1f);
    move_servo(_tilt_idx, _tilt_angle*10, _tilt_angle_min*0.1f, _tilt_angle_max*0.1f);
    move_servo(_pan_idx,  _pan_angle*10,  _pan_angle_min*0.1f,  _pan_angle_max*0.1f);
}

void AP_Mount::set_mode(enum MAV_MOUNT_MODE mode)
{
    _mount_mode = (int8_t)mode;
}

/// Change the configuration of the mount
/// triggered by a MavLink packet.
void AP_Mount::configure_msg(mavlink_message_t* msg)
{
    __mavlink_mount_configure_t packet;
    mavlink_msg_mount_configure_decode(msg, &packet);
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        // not for us
        return;
    }
    set_mode((enum MAV_MOUNT_MODE)packet.mount_mode);
    _stab_roll  = packet.stab_roll;
    _stab_tilt  = packet.stab_pitch;
    _stab_pan   = packet.stab_yaw;
}

/// Control the mount (depends on the previously set mount configuration)
/// triggered by a MavLink packet.
void AP_Mount::control_msg(mavlink_message_t *msg)
{
    __mavlink_mount_control_t packet;
    mavlink_msg_mount_control_decode(msg, &packet);
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        // not for us
        return;
    }

    switch ((enum MAV_MOUNT_MODE)_mount_mode.get())
    {
#if MNT_RETRACT_OPTION == ENABLED
    case MAV_MOUNT_MODE_RETRACT:      // Load and keep safe position (Roll,Pitch,Yaw) from EEPROM and stop stabilization
        set_retract_angles(packet.input_b*0.01f, packet.input_a*0.01f, packet.input_c*0.01f);
        if (packet.save_position)
        {
            _retract_angles.save();
        }
        break;
#endif

    case MAV_MOUNT_MODE_NEUTRAL:      //  Load and keep neutral position (Roll,Pitch,Yaw) from EEPROM
        set_neutral_angles(packet.input_b*0.01f, packet.input_a*0.01f, packet.input_c*0.01f);
        if (packet.save_position)
        {
            _neutral_angles.save();
        }
        break;

    case MAV_MOUNT_MODE_MAVLINK_TARGETING:      // Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
        set_control_angles(packet.input_b*0.01f, packet.input_a*0.01f, packet.input_c*0.01f);
        break;

    case MAV_MOUNT_MODE_RC_TARGETING:      // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
    {
        Vector3f vec = _neutral_angles.get();
        _roll_angle  = vec.x;
        _tilt_angle = vec.y;
        _pan_angle   = vec.z;
    }
    break;

#if MNT_GPSPOINT_OPTION == ENABLED
    case MAV_MOUNT_MODE_GPS_POINT:      // Load neutral position and start to point to Lat,Lon,Alt
        Location targetGPSLocation;
        targetGPSLocation.lat = packet.input_a;
        targetGPSLocation.lng = packet.input_b;
        targetGPSLocation.alt = packet.input_c;
        set_GPS_target_location(targetGPSLocation);
        break;
#endif

    case MAV_MOUNT_MODE_ENUM_END:
        break;

    default:
        // do nothing
        break;
    }
}

/// Return mount status information (depends on the previously set mount configuration)
/// triggered by a MavLink packet.
void AP_Mount::status_msg(mavlink_message_t *msg)
{
    __mavlink_mount_status_t packet;
    mavlink_msg_mount_status_decode(msg, &packet);
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        // not for us
        return;
    }

    switch ((enum MAV_MOUNT_MODE)_mount_mode.get())
    {
    case MAV_MOUNT_MODE_RETRACT:                        // safe position (Roll,Pitch,Yaw) from EEPROM and stop stabilization
    case MAV_MOUNT_MODE_NEUTRAL:                        // neutral position (Roll,Pitch,Yaw) from EEPROM
    case MAV_MOUNT_MODE_MAVLINK_TARGETING:      // neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
    case MAV_MOUNT_MODE_RC_TARGETING:                   // neutral position and start RC Roll,Pitch,Yaw control with stabilization
        packet.pointing_b = _roll_angle*100;            // degrees*100
        packet.pointing_a = _tilt_angle*100;            // degrees*100
        packet.pointing_c = _pan_angle*100;             // degrees*100
        break;
#if MNT_GPSPOINT_OPTION == ENABLED
    case MAV_MOUNT_MODE_GPS_POINT:             // neutral position and start to point to Lat,Lon,Alt
        packet.pointing_a = _target_GPS_location.lat;   // latitude
        packet.pointing_b = _target_GPS_location.lng;   // longitude
        packet.pointing_c = _target_GPS_location.alt;   // altitude
        break;
#endif
    case MAV_MOUNT_MODE_ENUM_END:
        break;
    }

    // status reply
    // TODO: is COMM_3 correct ?
    mavlink_msg_mount_status_send(MAVLINK_COMM_3, packet.target_system, packet.target_component,
                                  packet.pointing_a, packet.pointing_b, packet.pointing_c);
}

/// Set mount point/region of interest, triggered by mission script commands
void AP_Mount::set_roi_cmd(const struct Location *target_loc)
{
#if MNT_GPSPOINT_OPTION == ENABLED
    // set the target gps location
    _target_GPS_location = *target_loc;

    // set the mode to GPS tracking mode
    set_mode(MAV_MOUNT_MODE_GPS_POINT);
#endif
}

/// Set mount configuration, triggered by mission script commands
void AP_Mount::configure_cmd()
{
    // TODO get the information out of the mission command and use it
}

/// Control the mount (depends on the previously set mount configuration), triggered by mission script commands
void AP_Mount::control_cmd()
{
    // TODO get the information out of the mission command and use it
}

/// returns the angle (degrees*100) that the RC_Channel input is receiving
int32_t
AP_Mount::angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return (rc->get_reverse() ? -1 : 1) * (rc->radio_in - rc->radio_min) * (int32_t)(angle_max - angle_min) / (rc->radio_max - rc->radio_min) + (rc->get_reverse() ? angle_max : angle_min);
}

/// returns the angle (radians) that the RC_Channel input is receiving
float
AP_Mount::angle_input_rad(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return radians(angle_input(rc, angle_min, angle_max)*0.01f);
}

void
AP_Mount::calc_GPS_target_angle(const struct Location *target)
{
    float GPS_vector_x = (target->lng-_current_loc->lng)*cosf(ToRad((_current_loc->lat+target->lat)*0.00000005f))*0.01113195f;
    float GPS_vector_y = (target->lat-_current_loc->lat)*0.01113195f;
    float GPS_vector_z = (target->alt-_current_loc->alt);                 // baro altitude(IN CM) should be adjusted to known home elevation before take off (Set altimeter).
    float target_distance = 100.0f*pythagorous2(GPS_vector_x, GPS_vector_y);      // Careful , centimeters here locally. Baro/alt is in cm, lat/lon is in meters.
    _roll_control_angle  = 0;
    _tilt_control_angle  = atan2f(GPS_vector_z, target_distance);
    _pan_control_angle   = atan2f(GPS_vector_x, GPS_vector_y);
}

/// Stabilizes mount relative to the Earth's frame
/// Inputs:
///    _roll_control_angle   desired roll       angle in radians,
///    _tilt_control_angle   desired tilt/pitch angle in radians,
///    _pan_control_angle    desired pan/yaw    angle in radians
/// Outputs:
///    _roll_angle           stabilized roll       angle in degrees,
///    _tilt_angle           stabilized tilt/pitch angle in degrees,
///    _pan_angle            stabilized pan/yaw    angle in degrees
void
AP_Mount::stabilize()
{
#if MNT_STABILIZE_OPTION == ENABLED
    if (_ahrs) {
        // only do the full 3D frame transform if we are doing pan control
        if (_stab_pan) {
            Matrix3f m;                         ///< holds 3 x 3 matrix, var is used as temp in calcs
            Matrix3f cam;                       ///< Rotation matrix earth to camera. Desired camera from input.
            Matrix3f gimbal_target;             ///< Rotation matrix from plane to camera. Then Euler angles to the servos.
            m = _ahrs->get_dcm_matrix();
            m.transpose();
            cam.from_euler(_roll_control_angle, _tilt_control_angle, _pan_control_angle);
            gimbal_target = m * cam;
            gimbal_target.to_euler(&_roll_angle, &_tilt_angle, &_pan_angle);
            _roll_angle  = _stab_roll ? degrees(_roll_angle) : degrees(_roll_control_angle);
            _tilt_angle  = _stab_tilt ? degrees(_tilt_angle) : degrees(_tilt_control_angle);
            _pan_angle   = degrees(_pan_angle);
        } else {
            // otherwise base mount roll and tilt on the ahrs
            // roll/tilt attitude, plus any requested angle
            _roll_angle  = degrees(_roll_control_angle);
            _tilt_angle  = degrees(_tilt_control_angle);
            _pan_angle   = degrees(_pan_control_angle);
            if (_stab_roll) {
                _roll_angle -= degrees(_ahrs->roll);
            }
            if (_stab_tilt) {
                _tilt_angle -= degrees(_ahrs->pitch);
            }
        }
    } else {
#endif
        _roll_angle  = degrees(_roll_control_angle);
        _tilt_angle  = degrees(_tilt_control_angle);
        _pan_angle   = degrees(_pan_control_angle);
#if MNT_STABILIZE_OPTION == ENABLED
    }
#endif
}
/*
 *  /// For testing and development. Called in the medium loop.
 *  void
 *  AP_Mount::debug_output()
 *  {   Serial3.print("current   -     ");
 *       Serial3.print("lat ");
 *       Serial3.print(_current_loc->lat);
 *       Serial3.print(",lon ");
 *       Serial3.print(_current_loc->lng);
 *       Serial3.print(",alt ");
 *       Serial3.println(_current_loc->alt);
 *
 *       Serial3.print("gps       -     ");
 *       Serial3.print("lat ");
 *       Serial3.print(_gps->latitude);
 *       Serial3.print(",lon ");
 *       Serial3.print(_gps->longitude);
 *       Serial3.print(",alt ");
 *       Serial3.print(_gps->altitude);
 *       Serial3.println();
 *
 *       Serial3.print("target   -      ");
 *       Serial3.print("lat ");
 *       Serial3.print(_target_GPS_location.lat);
 *       Serial3.print(",lon ");
 *       Serial3.print(_target_GPS_location.lng);
 *       Serial3.print(",alt ");
 *       Serial3.print(_target_GPS_location.alt);
 *       Serial3.print(" hdg to targ ");
 *       Serial3.print(degrees(_pan_control_angle));
 *       Serial3.println();
 *  }
 */
/// saturate to the closest angle limit if outside of [min max] angle interval
/// input angle is in degrees * 10
int16_t
AP_Mount::closest_limit(int16_t angle, int16_t* angle_min, int16_t* angle_max)
{
    // Make sure the angle lies in the interval [-180 .. 180[ degrees
    while (angle < -1800) angle += 3600;
    while (angle >= 1800) angle -= 3600;

    // Make sure the angle limits lie in the interval [-180 .. 180[ degrees
    while (*angle_min < -1800) *angle_min += 3600;
    while (*angle_min >= 1800) *angle_min -= 3600;
    while (*angle_max < -1800) *angle_max += 3600;
    while (*angle_max >= 1800) *angle_max -= 3600;
    // TODO call this function somehow, otherwise this will never work
    //set_range(min, max);

    // If the angle is outside servo limits, saturate the angle to the closest limit
    // On a circle the closest angular position must be carefully calculated to account for wrap-around
    if ((angle < *angle_min) && (angle > *angle_max)) {
        // angle error if min limit is used
        int16_t err_min = *angle_min - angle + (angle<*angle_min ? 0 : 3600);     // add 360 degrees if on the "wrong side"
        // angle error if max limit is used
        int16_t err_max = angle - *angle_max + (angle>*angle_max ? 0 : 3600);     // add 360 degrees if on the "wrong side"
        angle = err_min<err_max ? *angle_min : *angle_max;
    }

    return angle;
}

/// all angles are degrees * 10 units
void
AP_Mount::move_servo(uint8_t function_idx, int16_t angle, int16_t angle_min, int16_t angle_max)
{
	// saturate to the closest angle limit if outside of [min max] angle interval
	int16_t servo_out = closest_limit(angle, &angle_min, &angle_max);
	RC_Channel_aux::move_servo((RC_Channel_aux::Aux_servo_function_t)function_idx, servo_out, angle_min, angle_max);
}
