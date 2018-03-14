#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "AP_Mount.h"
#include "AP_Mount_Backend.h"
#include "AP_Mount_Servo.h"
#include "AP_Mount_SoloGimbal.h"
#include "AP_Mount_Alexmos.h"
#include "AP_Mount_SToRM32.h"
#include "AP_Mount_SToRM32_serial.h"

const AP_Param::GroupInfo AP_Mount::var_info[] = {
    // @Param: _DEFLT_MODE
    // @DisplayName: Mount default operating mode
    // @Description: Mount default operating mode on startup and after control is returned from autopilot
    // @Values: 0:Retracted,1:Neutral,2:MavLink Targeting,3:RC Targeting,4:GPS Point
    // @User: Standard
    AP_GROUPINFO("_DEFLT_MODE", 0, AP_Mount, state[0]._default_mode, MAV_MOUNT_MODE_RC_TARGETING),

    // @Param: _RETRACT_X
    // @DisplayName: Mount roll angle when in retracted position
    // @Description: Mount roll angle when in retracted position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: _RETRACT_Y
    // @DisplayName: Mount tilt/pitch angle when in retracted position
    // @Description: Mount tilt/pitch angle when in retracted position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: _RETRACT_Z
    // @DisplayName: Mount yaw/pan angle when in retracted position
    // @Description: Mount yaw/pan angle when in retracted position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_RETRACT",    1, AP_Mount, state[0]._retract_angles, 0),

    // @Param: _NEUTRAL_X
    // @DisplayName: Mount roll angle when in neutral position
    // @Description: Mount roll angle when in neutral position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: _NEUTRAL_Y
    // @DisplayName: Mount tilt/pitch angle when in neutral position
    // @Description: Mount tilt/pitch angle when in neutral position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: _NEUTRAL_Z
    // @DisplayName: Mount pan/yaw angle when in neutral position
    // @Description: Mount pan/yaw angle when in neutral position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_NEUTRAL",    2, AP_Mount, state[0]._neutral_angles, 0),

    // 3 was used for control_angles

    // @Param: _STAB_ROLL
    // @DisplayName: Stabilize mount's roll angle
    // @Description: enable roll stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_STAB_ROLL",  4, AP_Mount, state[0]._stab_roll, 0),

    // @Param: _STAB_TILT
    // @DisplayName: Stabilize mount's pitch/tilt angle
    // @Description: enable tilt/pitch stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_STAB_TILT", 5, AP_Mount, state[0]._stab_tilt,  0),

    // @Param: _STAB_PAN
    // @DisplayName: Stabilize mount pan/yaw angle
    // @Description: enable pan/yaw stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_STAB_PAN",   6, AP_Mount, state[0]._stab_pan,  0),

    // @Param: _RC_IN_ROLL
    // @DisplayName: roll RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control roll movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8,9:RC9,10:RC10,11:RC11,12:RC12
    // @User: Standard
    AP_GROUPINFO("_RC_IN_ROLL",  7, AP_Mount, state[0]._roll_rc_in, 0),

    // @Param: _ANGMIN_ROL
    // @DisplayName: Minimum roll angle
    // @Description: Minimum physical roll angular position of mount.
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ANGMIN_ROL", 8, AP_Mount, state[0]._roll_angle_min, -4500),

    // @Param: _ANGMAX_ROL
    // @DisplayName: Maximum roll angle
    // @Description: Maximum physical roll angular position of the mount
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ANGMAX_ROL", 9, AP_Mount, state[0]._roll_angle_max, 4500),

    // @Param: _RC_IN_TILT
    // @DisplayName: tilt (pitch) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8,9:RC9,10:RC10,11:RC11,12:RC12
    // @User: Standard
    AP_GROUPINFO("_RC_IN_TILT",  10, AP_Mount, state[0]._tilt_rc_in,    0),

    // @Param: _ANGMIN_TIL
    // @DisplayName: Minimum tilt angle
    // @Description: Minimum physical tilt (pitch) angular position of mount.
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ANGMIN_TIL", 11, AP_Mount, state[0]._tilt_angle_min, -4500),

    // @Param: _ANGMAX_TIL
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum physical tilt (pitch) angular position of the mount
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ANGMAX_TIL", 12, AP_Mount, state[0]._tilt_angle_max, 4500),

    // @Param: _RC_IN_PAN
    // @DisplayName: pan (yaw) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control pan (yaw) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8,9:RC9,10:RC10,11:RC11,12:RC12
    // @User: Standard
    AP_GROUPINFO("_RC_IN_PAN",  13, AP_Mount, state[0]._pan_rc_in,       0),

    // @Param: _ANGMIN_PAN
    // @DisplayName: Minimum pan angle
    // @Description: Minimum physical pan (yaw) angular position of mount.
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ANGMIN_PAN",  14, AP_Mount, state[0]._pan_angle_min,  -4500),

    // @Param: _ANGMAX_PAN
    // @DisplayName: Maximum pan angle
    // @Description: Maximum physical pan (yaw) angular position of the mount
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ANGMAX_PAN",  15, AP_Mount, state[0]._pan_angle_max,  4500),

    // @Param: _JSTICK_SPD
    // @DisplayName: mount joystick speed
    // @Description: 0 for position control, small for low speeds, 100 for max speed. A good general value is 10 which gives a movement speed of 3 degrees per second.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_JSTICK_SPD",  16, AP_Mount, _joystick_speed, 0),

    // @Param: _LEAD_RLL
    // @DisplayName: Roll stabilization lead time
    // @Description: Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate, compensating for servo delay. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
    // @Units: s
    // @Range: 0.0 0.2
    // @Increment: .005
    // @User: Standard
    AP_GROUPINFO("_LEAD_RLL", 17, AP_Mount, state[0]._roll_stb_lead, 0.0f),

    // @Param: _LEAD_PTCH
    // @DisplayName: Pitch stabilization lead time
    // @Description: Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
    // @Units: s
    // @Range: 0.0 0.2
    // @Increment: .005
    // @User: Standard
    AP_GROUPINFO("_LEAD_PTCH", 18, AP_Mount, state[0]._pitch_stb_lead, 0.0f),

    // @Param: _TYPE
    // @DisplayName: Mount Type
    // @Description: Mount Type (None, Servo or MAVLink)
    // @Values: 0:None, 1:Servo, 2:3DR Solo, 3:Alexmos Serial, 4:SToRM32 MAVLink, 5:SToRM32 Serial
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_TYPE", 19, AP_Mount, state[0]._type, 0),

    // 20 formerly _OFF_JNT

    // 21 formerly _OFF_ACC

    // 22 formerly _OFF_GYRO

    // 23 formerly _K_RATE

    // 24 is AVAILABLE

#if AP_MOUNT_MAX_INSTANCES > 1
    // @Param: 2_DEFLT_MODE
    // @DisplayName: Mount default operating mode
    // @Description: Mount default operating mode on startup and after control is returned from autopilot
    // @Values: 0:Retracted,1:Neutral,2:MavLink Targeting,3:RC Targeting,4:GPS Point
    // @User: Standard
    AP_GROUPINFO("2_DEFLT_MODE",    25, AP_Mount, state[1]._default_mode, MAV_MOUNT_MODE_RC_TARGETING),

    // @Param: 2_RETRACT_X
    // @DisplayName: Mount2 roll angle when in retracted position
    // @Description: Mount2 roll angle when in retracted position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: 2_RETRACT_Y
    // @DisplayName: Mount2 tilt/pitch angle when in retracted position
    // @Description: Mount2 tilt/pitch angle when in retracted position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: 2_RETRACT_Z
    // @DisplayName: Mount2 yaw/pan angle when in retracted position
    // @Description: Mount2 yaw/pan angle when in retracted position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("2_RETRACT",       26, AP_Mount, state[1]._retract_angles, 0),

    // @Param: 2_NEUTRAL_X
    // @DisplayName: Mount2 roll angle when in neutral position
    // @Description: Mount2 roll angle when in neutral position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: 2_NEUTRAL_Y
    // @DisplayName: Mount2 tilt/pitch angle when in neutral position
    // @Description: Mount2 tilt/pitch angle when in neutral position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: 2_NEUTRAL_Z
    // @DisplayName: Mount2 pan/yaw angle when in neutral position
    // @Description: Mount2 pan/yaw angle when in neutral position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("2_NEUTRAL",       27, AP_Mount, state[1]._neutral_angles, 0),

    // 3 was used for control_angles

    // @Param: 2_STAB_ROLL
    // @DisplayName: Stabilize Mount2's roll angle
    // @Description: enable roll stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("2_STAB_ROLL",     28, AP_Mount, state[1]._stab_roll, 0),

    // @Param: 2_STAB_TILT
    // @DisplayName: Stabilize Mount2's pitch/tilt angle
    // @Description: enable tilt/pitch stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("2_STAB_TILT",     29, AP_Mount, state[1]._stab_tilt,  0),

    // @Param: 2_STAB_PAN
    // @DisplayName: Stabilize mount2 pan/yaw angle
    // @Description: enable pan/yaw stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("2_STAB_PAN",      30, AP_Mount, state[1]._stab_pan,  0),

    // @Param: 2_RC_IN_ROLL
    // @DisplayName: Mount2's roll RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control roll movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8,9:RC9,10:RC10,11:RC11,12:RC12
    // @User: Standard
    AP_GROUPINFO("2_RC_IN_ROLL",    31, AP_Mount, state[1]._roll_rc_in, 0),

    // @Param: 2_ANGMIN_ROL
    // @DisplayName: Mount2's minimum roll angle
    // @Description: Mount2's minimum physical roll angular position
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("2_ANGMIN_ROL",    32, AP_Mount, state[1]._roll_angle_min, -4500),

    // @Param: 2_ANGMAX_ROL
    // @DisplayName: Mount2's maximum roll angle
    // @Description: Mount2's maximum physical roll angular position
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("2_ANGMAX_ROL",    33, AP_Mount, state[1]._roll_angle_max, 4500),

    // @Param: 2_RC_IN_TILT
    // @DisplayName: Mount2's tilt (pitch) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8,9:RC9,10:RC10,11:RC11,12:RC12
    // @User: Standard
    AP_GROUPINFO("2_RC_IN_TILT",    34, AP_Mount, state[1]._tilt_rc_in,    0),

    // @Param: 2_ANGMIN_TIL
    // @DisplayName: Mount2's minimum tilt angle
    // @Description: Mount2's minimum physical tilt (pitch) angular position
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("2_ANGMIN_TIL",    35, AP_Mount, state[1]._tilt_angle_min, -4500),

    // @Param: 2_ANGMAX_TIL
    // @DisplayName: Mount2's maximum tilt angle
    // @Description: Mount2's maximum physical tilt (pitch) angular position
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("2_ANGMAX_TIL",    36, AP_Mount, state[1]._tilt_angle_max, 4500),

    // @Param: 2_RC_IN_PAN
    // @DisplayName: Mount2's pan (yaw) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control pan (yaw) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8,9:RC9,10:RC10,11:RC11,12:RC12
    // @User: Standard
    AP_GROUPINFO("2_RC_IN_PAN",     37, AP_Mount, state[1]._pan_rc_in,       0),

    // @Param: 2_ANGMIN_PAN
    // @DisplayName: Mount2's minimum pan angle
    // @Description: Mount2's minimum physical pan (yaw) angular position
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("2_ANGMIN_PAN",    38, AP_Mount, state[1]._pan_angle_min,  -4500),

    // @Param: 2_ANGMAX_PAN
    // @DisplayName: Mount2's maximum pan angle
    // @Description: MOunt2's maximum physical pan (yaw) angular position
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("2_ANGMAX_PAN",    39, AP_Mount, state[1]._pan_angle_max,  4500),

    // @Param: 2_LEAD_RLL
    // @DisplayName: Mount2's Roll stabilization lead time
    // @Description: Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate, compensating for servo delay. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
    // @Units: s
    // @Range: 0.0 0.2
    // @Increment: .005
    // @User: Standard
    AP_GROUPINFO("2_LEAD_RLL",      40, AP_Mount, state[1]._roll_stb_lead, 0.0f),

    // @Param: 2_LEAD_PTCH
    // @DisplayName: Mount2's Pitch stabilization lead time
    // @Description: Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
    // @Units: s
    // @Range: 0.0 0.2
    // @Increment: .005
    // @User: Standard
    AP_GROUPINFO("2_LEAD_PTCH",      41, AP_Mount, state[1]._pitch_stb_lead, 0.0f),

    // @Param: 2_TYPE
    // @DisplayName: Mount2 Type
    // @Description: Mount Type (None, Servo or MAVLink)
    // @Values: 0:None, 1:Servo, 2:3DR Solo, 3:Alexmos Serial, 4:SToRM32 MAVLink, 5:SToRM32 Serial
    // @User: Standard
    AP_GROUPINFO("2_TYPE",           42, AP_Mount, state[1]._type, 0),
#endif // AP_MOUNT_MAX_INSTANCES > 1

    AP_GROUPEND
};

AP_Mount::AP_Mount(const AP_AHRS_TYPE &ahrs, const struct Location &current_loc) :
    _ahrs(ahrs),
    _current_loc(current_loc),
    _num_instances(0),
    _primary(0)
{
	AP_Param::setup_object_defaults(this, var_info);

    // initialise backend pointers and mode
    for (uint8_t i=0; i<AP_MOUNT_MAX_INSTANCES; i++) {
        _backends[i] = nullptr;
    }
}

// init - detect and initialise all mounts
void AP_Mount::init(const AP_SerialManager& serial_manager)
{
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    // default mount to servo mount if rc output channels to control roll, tilt or pan have been defined
    if (!state[0]._type.configured()) {
        if (SRV_Channels::function_assigned(SRV_Channel::Aux_servo_function_t::k_mount_pan) ||
            SRV_Channels::function_assigned(SRV_Channel::Aux_servo_function_t::k_mount_tilt) ||
            SRV_Channels::function_assigned(SRV_Channel::Aux_servo_function_t::k_mount_roll)) {
                state[0]._type.set_and_save(Mount_Type_Servo);
        }
    }

    // primary is reset to the first instantiated mount
    bool primary_set = false;

    // create each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        // default instance's state
        state[instance]._mode = (enum MAV_MOUNT_MODE)state[instance]._default_mode.get();

        MountType mount_type = get_mount_type(instance);

        // check for servo mounts
        if (mount_type == Mount_Type_Servo) {
            _backends[instance] = new AP_Mount_Servo(*this, state[instance], instance);
            _num_instances++;

#if AP_AHRS_NAVEKF_AVAILABLE
#if !HAL_MINIMIZE_FEATURES
        // check for MAVLink mounts
        } else if (mount_type == Mount_Type_SoloGimbal) {
            _backends[instance] = new AP_Mount_SoloGimbal(*this, state[instance], instance);
            _num_instances++;
#endif // HAL_MINIMIZE_FEATURES
#endif // AP_AHRS_NAVEKF_AVAILABLE

        // check for Alexmos mounts
        } else if (mount_type == Mount_Type_Alexmos) {
            _backends[instance] = new AP_Mount_Alexmos(*this, state[instance], instance);
            _num_instances++;

        // check for SToRM32 mounts using MAVLink protocol
        } else if (mount_type == Mount_Type_SToRM32) {
            _backends[instance] = new AP_Mount_SToRM32(*this, state[instance], instance);
            _num_instances++;

        // check for SToRM32 mounts using serial protocol
        } else if (mount_type == Mount_Type_SToRM32_serial) {
            _backends[instance] = new AP_Mount_SToRM32_serial(*this, state[instance], instance);
            _num_instances++;
        }

        // init new instance
        if (_backends[instance] != nullptr) {
            _backends[instance]->init(serial_manager);
            if (!primary_set) {
                _primary = instance;
                primary_set = true;
            }
        }
    }
}

// update - give mount opportunity to update servos.  should be called at 10hz or higher
void AP_Mount::update()
{
    // update each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->update();
        }
    }
}

// used for gimbals that need to read INS data at full rate
void AP_Mount::update_fast()
{
    // update each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->update_fast();
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
    if (instance >= AP_MOUNT_MAX_INSTANCES || _backends[instance] == nullptr) {
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

    return state[instance]._mode;
}

// set_mode_to_default - restores the mode to it's default mode held in the MNT_MODE parameter
//      this operation requires 60us on a Pixhawk/PX4
void AP_Mount::set_mode_to_default(uint8_t instance)
{
    set_mode(instance, (enum MAV_MOUNT_MODE)state[instance]._default_mode.get());
}

// set_mode - sets mount's mode
void AP_Mount::set_mode(uint8_t instance, enum MAV_MOUNT_MODE mode)
{
    // sanity check instance
    if (instance >= AP_MOUNT_MAX_INSTANCES || _backends[instance] == nullptr) {
        return;
    }

    // call backend's set_mode
    _backends[instance]->set_mode(mode);
}

// set_angle_targets - sets angle targets in degrees
void AP_Mount::set_angle_targets(uint8_t instance, float roll, float tilt, float pan)
{
    if (instance >= AP_MOUNT_MAX_INSTANCES || _backends[instance] == nullptr) {
        return;
    }

    // send command to backend
    _backends[instance]->set_angle_targets(roll, tilt, pan);
}

/// Change the configuration of the mount
/// triggered by a MavLink packet.
void AP_Mount::configure_msg(uint8_t instance, mavlink_message_t* msg)
{
    if (instance >= AP_MOUNT_MAX_INSTANCES || _backends[instance] == nullptr) {
        return;
    }

    // send message to backend
    _backends[instance]->configure_msg(msg);
}

/// Control the mount (depends on the previously set mount configuration)
/// triggered by a MavLink packet.
void AP_Mount::control_msg(uint8_t instance, mavlink_message_t *msg)
{
    if (instance >= AP_MOUNT_MAX_INSTANCES || _backends[instance] == nullptr) {
        return;
    }

    // send message to backend
    _backends[instance]->control_msg(msg);
}

void AP_Mount::control(uint8_t instance, int32_t pitch_or_lat, int32_t roll_or_lon, int32_t yaw_or_alt, enum MAV_MOUNT_MODE mount_mode)
{
    if (instance >= AP_MOUNT_MAX_INSTANCES || _backends[instance] == nullptr) {
        return;
    }

    // send message to backend
    _backends[instance]->control(pitch_or_lat, roll_or_lon, yaw_or_alt, mount_mode);
}

/// Return mount status information
void AP_Mount::status_msg(mavlink_channel_t chan)
{
    // call status_msg for  each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->status_msg(chan);
        }
    }
}

// set_roi_target - sets target location that mount should attempt to point towards
void AP_Mount::set_roi_target(uint8_t instance, const struct Location &target_loc)
{
    // call instance's set_roi_cmd
    if (instance < AP_MOUNT_MAX_INSTANCES && _backends[instance] != nullptr) {
        _backends[instance]->set_roi_target(target_loc);
    }
}

// pass a GIMBAL_REPORT message to the backend
void AP_Mount::handle_gimbal_report(mavlink_channel_t chan, mavlink_message_t *msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_gimbal_report(chan, msg);
        }
    }
}

// handle PARAM_VALUE
void AP_Mount::handle_param_value(mavlink_message_t *msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_param_value(msg);
        }
    }
}

// send a GIMBAL_REPORT message to the GCS
void AP_Mount::send_gimbal_report(mavlink_channel_t chan)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_gimbal_report(chan);
        }
    }    
}
