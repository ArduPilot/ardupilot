// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Mount.h>
#include <AP_Mount_Backend.h>
#include <AP_Mount_Servo.h>
#include <AP_Mount_MAVLink.h>

const AP_Param::GroupInfo AP_Mount::var_info[] PROGMEM = {
    // @Param: MODE
    // @DisplayName: Mount operation mode
    // @Description: Camera or antenna mount operation mode
    // @Values: 0:retract,1:neutral,2:MavLink_targeting,3:RC_targeting,4:GPS_point
    // @User: Standard
    AP_GROUPINFO("MODE",       0, AP_Mount, state[0]._mode, MAV_MOUNT_MODE_RETRACT), // see MAV_MOUNT_MODE at ardupilotmega.h

    // @Param: RETRACT_X
    // @DisplayName: Mount roll angle when in retracted position
    // @Description: Mount roll angle when in retracted position
    // @Units: Degrees
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: RETRACT_Y
    // @DisplayName: Mount tilt/pitch angle when in retracted position
    // @Description: Mount tilt/pitch angle when in retracted position
    // @Units: Degrees
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: RETRACT_Z
    // @DisplayName: Mount yaw/pan angle when in retracted position
    // @Description: Mount yaw/pan angle when in retracted position
    // @Units: Degrees
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RETRACT",    1, AP_Mount, state[0]._retract_angles, 0),

    // @Param: NEUTRAL_X
    // @DisplayName: Mount roll angle when in neutral position
    // @Description: Mount roll angle when in neutral position
    // @Units: Degrees
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: NEUTRAL_Y
    // @DisplayName: Mount tilt/pitch angle when in neutral position
    // @Description: Mount tilt/pitch angle when in neutral position
    // @Units: Degrees
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: NEUTRAL_Z
    // @DisplayName: Mount pan/yaw angle when in neutral position
    // @Description: Mount pan/yaw angle when in neutral position
    // @Units: Degrees
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("NEUTRAL",    2, AP_Mount, state[0]._neutral_angles, 0),

    // 3 was used for control_angles

    // @Param: STAB_ROLL
    // @DisplayName: Stabilize mount's roll angle
    // @Description: enable roll stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("STAB_ROLL",  4, AP_Mount, state[0]._stab_roll, 0),

    // @Param: STAB_TILT
    // @DisplayName: Stabilize mount's pitch/tilt angle
    // @Description: enable tilt/pitch stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("STAB_TILT", 5, AP_Mount, state[0]._stab_tilt,  0),

    // @Param: STAB_PAN
    // @DisplayName: Stabilize mount pan/yaw angle
    // @Description: enable pan/yaw stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("STAB_PAN",   6, AP_Mount, state[0]._stab_pan,  0),

    // @Param: RC_IN_ROLL
    // @DisplayName: roll RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control roll movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8
    // @User: Standard
    AP_GROUPINFO("RC_IN_ROLL",  7, AP_Mount, state[0]._roll_rc_in, 0),

    // @Param: ANGMIN_ROL
    // @DisplayName: Minimum roll angle
    // @Description: Minimum physical roll angular position of mount.
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_ROL", 8, AP_Mount, state[0]._roll_angle_min, -4500),

    // @Param: ANGMAX_ROL
    // @DisplayName: Maximum roll angle
    // @Description: Maximum physical roll angular position of the mount
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_ROL", 9, AP_Mount, state[0]._roll_angle_max, 4500),

    // @Param: RC_IN_TILT
    // @DisplayName: tilt (pitch) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8
    // @User: Standard
    AP_GROUPINFO("RC_IN_TILT",  10, AP_Mount, state[0]._tilt_rc_in,    0),

    // @Param: ANGMIN_TIL
    // @DisplayName: Minimum tilt angle
    // @Description: Minimum physical tilt (pitch) angular position of mount.
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_TIL", 11, AP_Mount, state[0]._tilt_angle_min, -4500),

    // @Param: ANGMAX_TIL
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum physical tilt (pitch) angular position of the mount
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_TIL", 12, AP_Mount, state[0]._tilt_angle_max, 4500),

    // @Param: RC_IN_PAN
    // @DisplayName: pan (yaw) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control pan (yaw) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8
    // @User: Standard
    AP_GROUPINFO("RC_IN_PAN",  13, AP_Mount, state[0]._pan_rc_in,       0),

    // @Param: ANGMIN_PAN
    // @DisplayName: Minimum pan angle
    // @Description: Minimum physical pan (yaw) angular position of mount.
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_PAN",  14, AP_Mount, state[0]._pan_angle_min,  -4500),

    // @Param: ANGMAX_PAN
    // @DisplayName: Maximum pan angle
    // @Description: Maximum physical pan (yaw) angular position of the mount
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_PAN",  15, AP_Mount, state[0]._pan_angle_max,  4500),

    // @Param: JSTICK_SPD
    // @DisplayName: mount joystick speed
    // @Description: 0 for position control, small for low speeds, 100 for max speed. A good general value is 10 which gives a movement speed of 3 degrees per second.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("JSTICK_SPD",  16, AP_Mount, _joystick_speed, 0),

    // @Param: LEAD_RLL
    // @DisplayName: Roll stabilization lead time
    // @Description: Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate, compensating for servo delay. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
    // @Units: Seconds
    // @Range: 0.0 0.2
    // @Increment: .005
    // @User: Standard
    AP_GROUPINFO("LEAD_RLL", 17, AP_Mount, state[0]._roll_stb_lead, 0.0f),

    // @Param: LEAD_PTCH
    // @DisplayName: Pitch stabilization lead time
    // @Description: Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
    // @Units: Seconds
    // @Range: 0.0 0.2
    // @Increment: .005
    // @User: Standard
    AP_GROUPINFO("LEAD_PTCH", 18, AP_Mount, state[0]._pitch_stb_lead, 0.0f),

    // @Param: TYPE
    // @DisplayName: Mount Type
    // @Description: Mount Type (None, Servo or MAVLink)
    // @Values: 0:None, 1:Servo, 2:MAVLink
    // @User: Standard
    AP_GROUPINFO("TYPE", 19, AP_Mount, state[0]._type, 0),

    AP_GROUPEND
};

AP_Mount::AP_Mount(const AP_AHRS &ahrs, const struct Location &current_loc) :
    _ahrs(ahrs),
    _current_loc(current_loc),
    _num_instances(0),
    _primary(0)
{
	AP_Param::setup_object_defaults(this, var_info);

    // initialise backend status
    for (uint8_t i=0; i<AP_MOUNT_MAX_INSTANCES; i++) {
        _backends[i] = NULL;
    }
}

// init - detect and initialise all mounts
void AP_Mount::init()
{
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    // create each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        MountType mount_type = get_mount_type(instance);

        // check for servo mounts
        if (mount_type == Mount_Type_Servo) {
            _backends[instance] = new AP_Mount_Servo(*this, instance);
            _num_instances++;

        // check for MAVLink mounts
        } else if (mount_type == Mount_Type_MAVLink) {
            _backends[instance] = new AP_Mount_MAVLink(*this, instance);
            _num_instances++;
        }

        // init new instance
        if (_backends[instance] != NULL) {
            _backends[instance]->init();
        }
    }
}

// update - give mount opportunity to update servos.  should be called at 10hz or higher
void AP_Mount::update()
{
    // update each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != NULL) {
            _backends[instance]->update();
        }
    }
}

// get_mount_type - returns the type of mount
AP_Mount::MountType AP_Mount::get_mount_type(uint8_t instance) const
{
    if (instance >= AP_MOUNT_MAX_INSTANCES) {
        return Mount_Type_None;
    }

    return (MountType)state[instance]._type.get();
}

// has_pan_control - returns true if the mount has yaw control (required for copters)
bool AP_Mount::has_pan_control(uint8_t instance) const
{
    if (instance >= AP_MOUNT_MAX_INSTANCES || _backends[instance] == NULL) {
        return false;
    }

    // ask backend if it support pan
    return _backends[instance]->has_pan_control();
}

// get_mode - returns current mode of mount (i.e. Retracted, Neutral, RC_Targeting, GPS Point)
MAV_MOUNT_MODE AP_Mount::get_mode(uint8_t instance) const
{
    // sanity check instance
    if (instance >= AP_MOUNT_MAX_INSTANCES) {
        return  MAV_MOUNT_MODE_RETRACT;
    }

    return (enum MAV_MOUNT_MODE)state[instance]._mode.get();
}

// set_mode_to_default - restores the mode to it's default mode held in the MNT_MODE parameter
//      this operation requires 230us on an APM2, 60us on a Pixhawk/PX4
void AP_Mount::set_mode_to_default(uint8_t instance)
{
    // sanity check instance
    if (instance >= AP_MOUNT_MAX_INSTANCES) {
        return;
    }

    // load instance's state from eeprom
    state[instance]._mode.load();
}

// set_mode - sets mount's mode
void AP_Mount::set_mode(uint8_t instance, enum MAV_MOUNT_MODE mode)
{
    // sanity check instance
    if (instance < AP_MOUNT_MAX_INSTANCES) {
        state[instance]._mode = (int8_t)mode;
    }
}

/// Change the configuration of the mount
/// triggered by a MavLink packet.
void AP_Mount::configure_msg(uint8_t instance, mavlink_message_t* msg)
{
    if (instance >= AP_MOUNT_MAX_INSTANCES || _backends[instance] == NULL) {
        return;
    }

    // send message to backend
    _backends[instance]->configure_msg(msg);
}

/// Control the mount (depends on the previously set mount configuration)
/// triggered by a MavLink packet.
void AP_Mount::control_msg(uint8_t instance, mavlink_message_t *msg)
{
    if (instance >= AP_MOUNT_MAX_INSTANCES || _backends[instance] == NULL) {
        return;
    }

    // send message to backend
    _backends[instance]->control_msg(msg);
}

/// Return mount status information
void AP_Mount::status_msg(mavlink_channel_t chan)
{
    // call status_msg for  each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != NULL) {
            _backends[instance]->status_msg(chan);
        }
    }
}

// set_roi_target - sets target location that mount should attempt to point towards
void AP_Mount::set_roi_target(uint8_t instance, const struct Location &target_loc)
{
    // call instance's set_roi_cmd
    if (instance < AP_MOUNT_MAX_INSTANCES && _backends[instance] != NULL) {
        _backends[instance]->set_roi_target(target_loc);
    }
}

