#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "AP_Mount.h"

#if HAL_MOUNT_ENABLED

#include "AP_Mount_Backend.h"
#include "AP_Mount_Servo.h"
#include "AP_Mount_SoloGimbal.h"
#include "AP_Mount_Alexmos.h"
#include "AP_Mount_SToRM32.h"
#include "AP_Mount_SToRM32_serial.h"
#include "AP_Mount_Gremsy.h"
#include <AP_Math/location.h>
#include <SRV_Channel/SRV_Channel.h>

const AP_Param::GroupInfo AP_Mount::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Mount Type
    // @Description: Mount Type (None, Servo or MAVLink)
    // @Values: 0:None, 1:Servo, 2:3DR Solo, 3:Alexmos Serial, 4:SToRM32 MAVLink, 5:SToRM32 Serial, 6:Gremsy
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE", 19, AP_Mount, state[0]._type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _DEFLT_MODE
    // @DisplayName: Mount default operating mode
    // @Description: Mount default operating mode on startup and after control is returned from autopilot
    // @Values: 0:Retracted,1:Neutral,2:MavLink Targeting,3:RC Targeting,4:GPS Point,6:Home Location
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
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("_ANGMIN_ROL", 8, AP_Mount, state[0]._roll_angle_min, -4500),

    // @Param: _ANGMAX_ROL
    // @DisplayName: Maximum roll angle
    // @Description: Maximum physical roll angular position of the mount
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 10
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
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("_ANGMIN_TIL", 11, AP_Mount, state[0]._tilt_angle_min, -4500),

    // @Param: _ANGMAX_TIL
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum physical tilt (pitch) angular position of the mount
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 10
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
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("_ANGMIN_PAN",  14, AP_Mount, state[0]._pan_angle_min,  -4500),

    // @Param: _ANGMAX_PAN
    // @DisplayName: Maximum pan angle
    // @Description: Maximum physical pan (yaw) angular position of the mount
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("_ANGMAX_PAN",  15, AP_Mount, state[0]._pan_angle_max,  4500),

    // 16 was _JSTICK_SPD

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

    // 19 _TYPE, now at top with enable flag

    // 20 formerly _OFF_JNT

    // 21 formerly _OFF_ACC

    // 22 formerly _OFF_GYRO

    // 23 formerly _K_RATE

    // @Param: _RC_RATE
    // @DisplayName: Mount RC Rate
    // @Description: Pilot rate control's maximum rate.  Set to zero to use angle control
    // @Units: deg/s
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_RC_RATE",  24, AP_Mount, _rc_rate_max, 0),

#if AP_MOUNT_MAX_INSTANCES > 1
    // @Param: 2_DEFLT_MODE
    // @DisplayName: Mount default operating mode
    // @Description: Mount default operating mode on startup and after control is returned from autopilot
    // @Values: 0:Retracted,1:Neutral,2:MavLink Targeting,3:RC Targeting,4:GPS Point,6:Home Location
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
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("2_ANGMIN_ROL",    32, AP_Mount, state[1]._roll_angle_min, -4500),

    // @Param: 2_ANGMAX_ROL
    // @DisplayName: Mount2's maximum roll angle
    // @Description: Mount2's maximum physical roll angular position
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 10
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
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("2_ANGMIN_TIL",    35, AP_Mount, state[1]._tilt_angle_min, -4500),

    // @Param: 2_ANGMAX_TIL
    // @DisplayName: Mount2's maximum tilt angle
    // @Description: Mount2's maximum physical tilt (pitch) angular position
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 10
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
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("2_ANGMIN_PAN",    38, AP_Mount, state[1]._pan_angle_min,  -4500),

    // @Param: 2_ANGMAX_PAN
    // @DisplayName: Mount2's maximum pan angle
    // @Description: MOunt2's maximum physical pan (yaw) angular position
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 10
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
    // @Values: 0:None, 1:Servo, 2:3DR Solo, 3:Alexmos Serial, 4:SToRM32 MAVLink, 5:SToRM32 Serial, 6:Gremsy
    // @User: Standard
    AP_GROUPINFO("2_TYPE",           42, AP_Mount, state[1]._type, 0),
#endif // AP_MOUNT_MAX_INSTANCES > 1

    AP_GROUPEND
};

AP_Mount::AP_Mount()
{
    if (_singleton != nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Mount must be singleton");
#endif
        return;
    }
    _singleton = this;

	AP_Param::setup_object_defaults(this, var_info);
}

// init - detect and initialise all mounts
void AP_Mount::init()
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

    // perform any required parameter conversion
    convert_params();

    // primary is reset to the first instantiated mount
    bool primary_set = false;

    // create each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        MountType mount_type = get_mount_type(instance);

        // check for servo mounts
        if (mount_type == Mount_Type_Servo) {
#if HAL_MOUNT_SERVO_ENABLED
            _backends[instance] = new AP_Mount_Servo(*this, state[instance], instance);
            _num_instances++;
#endif

#if HAL_SOLO_GIMBAL_ENABLED
        // check for Solo mounts
        } else if (mount_type == Mount_Type_SoloGimbal) {
            _backends[instance] = new AP_Mount_SoloGimbal(*this, state[instance], instance);
            _num_instances++;
#endif // HAL_SOLO_GIMBAL_ENABLED

#if HAL_MOUNT_ALEXMOS_ENABLED
        // check for Alexmos mounts
        } else if (mount_type == Mount_Type_Alexmos) {
            _backends[instance] = new AP_Mount_Alexmos(*this, state[instance], instance);
            _num_instances++;
#endif

#if HAL_MOUNT_STORM32MAVLINK_ENABLED
        // check for SToRM32 mounts using MAVLink protocol
        } else if (mount_type == Mount_Type_SToRM32) {
            _backends[instance] = new AP_Mount_SToRM32(*this, state[instance], instance);
            _num_instances++;
#endif

#if HAL_MOUNT_STORM32SERIAL_ENABLED
        // check for SToRM32 mounts using serial protocol
        } else if (mount_type == Mount_Type_SToRM32_serial) {
            _backends[instance] = new AP_Mount_SToRM32_serial(*this, state[instance], instance);
            _num_instances++;
#endif

#if HAL_MOUNT_GREMSY_ENABLED
        // check for Gremsy mounts
        } else if (mount_type == Mount_Type_Gremsy) {
            _backends[instance] = new AP_Mount_Gremsy(*this, state[instance], instance);
            _num_instances++;
#endif // HAL_MOUNT_GREMSY_ENABLED
        }

        // init new instance
        if (_backends[instance] != nullptr) {
            if (!primary_set) {
                _primary = instance;
                primary_set = true;
            }
        }
    }

    // init each instance, do it after all instances were created, so that they all know things
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->init();
            set_mode_to_default(instance);
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
    if (!check_instance(instance)) {
        return false;
    }

    // ask backend if it support pan
    return _backends[instance]->has_pan_control();
}

// get_mode - returns current mode of mount (i.e. Retracted, Neutral, RC_Targeting, GPS Point)
MAV_MOUNT_MODE AP_Mount::get_mode(uint8_t instance) const
{
    // sanity check instance
    if (!check_instance(instance)) {
        return MAV_MOUNT_MODE_RETRACT;
    }

    // ask backend its mode
    return _backends[instance]->get_mode();
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
    if (!check_instance(instance)) {
        return;
    }

    // call backend's set_mode
    _backends[instance]->set_mode(mode);
}

// set yaw_lock.  If true, the gimbal's yaw target is maintained in earth-frame meaning it will lock onto an earth-frame heading (e.g. North)
// If false (aka "follow") the gimbal's yaw is maintained in body-frame meaning it will rotate with the vehicle
void AP_Mount::set_yaw_lock(uint8_t instance, bool yaw_lock)
{
    // sanity check instance
    if (!check_instance(instance)) {
        return;
    }

    // call backend's set_yaw_lock
    _backends[instance]->set_yaw_lock(yaw_lock);
}

// set angle target in degrees
// yaw_is_earth_frame (aka yaw_lock) should be true if yaw angle is earth-frame, false if body-frame
void AP_Mount::set_angle_target(uint8_t instance, float roll_deg, float pitch_deg, float yaw_deg, bool yaw_is_earth_frame)
{
    if (!check_instance(instance)) {
        return;
    }

    // send command to backend
    _backends[instance]->set_angle_target(roll_deg, pitch_deg, yaw_deg, yaw_is_earth_frame);
}

// sets rate target in deg/s
// yaw_lock should be true if the yaw rate is earth-frame, false if body-frame (e.g. rotates with body of vehicle)
void AP_Mount::set_rate_target(uint8_t instance, float roll_degs, float pitch_degs, float yaw_degs, bool yaw_lock)
{
    if (!check_instance(instance)) {
        return;
    }

    // send command to backend
    _backends[instance]->set_rate_target(roll_degs, pitch_degs, yaw_degs, yaw_lock);
}

MAV_RESULT AP_Mount::handle_command_do_mount_configure(const mavlink_command_long_t &packet)
{
    if (!check_primary()) {
        return MAV_RESULT_FAILED;
    }
    _backends[_primary]->set_mode((MAV_MOUNT_MODE)packet.param1);
    state[_primary]._stab_roll.set(packet.param2);
    state[_primary]._stab_tilt.set(packet.param3);
    state[_primary]._stab_pan.set(packet.param4);

    return MAV_RESULT_ACCEPTED;
}


MAV_RESULT AP_Mount::handle_command_do_mount_control(const mavlink_command_long_t &packet)
{
    if (!check_primary()) {
        return MAV_RESULT_FAILED;
    }

    // send message to backend
    _backends[_primary]->control(packet.param1, packet.param2, packet.param3, (MAV_MOUNT_MODE) packet.param7);

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT AP_Mount::handle_command_do_gimbal_manager_pitchyaw(const mavlink_command_long_t &packet)
{
    if (!check_primary()) {
        return MAV_RESULT_FAILED;
    }

    // check flags for change to RETRACT
    uint32_t flags = (uint32_t)packet.param5;
    if ((flags & GIMBAL_MANAGER_FLAGS_RETRACT) > 0) {
        _backends[_primary]->set_mode(MAV_MOUNT_MODE_RETRACT);
        return MAV_RESULT_ACCEPTED;
    }
    // check flags for change to NEUTRAL
    if ((flags & GIMBAL_MANAGER_FLAGS_NEUTRAL) > 0) {
        _backends[_primary]->set_mode(MAV_MOUNT_MODE_NEUTRAL);
        return MAV_RESULT_ACCEPTED;
    }

    // To-Do: handle gimbal device id

    // param1 : pitch_angle (in degrees)
    // param2 : yaw angle (in degrees)
    const float pitch_angle_deg = packet.param1;
    const float yaw_angle_deg = packet.param2;
    if (!isnan(pitch_angle_deg) && !isnan(yaw_angle_deg)) {
        set_angle_target(0, pitch_angle_deg, yaw_angle_deg, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return MAV_RESULT_ACCEPTED;
    }

    // param3 : pitch_rate (in deg/s)
    // param4 : yaw rate (in deg/s)
    const float pitch_rate_degs = packet.param3;
    const float yaw_rate_degs = packet.param4;
    if (!isnan(pitch_rate_degs) && !isnan(yaw_rate_degs)) {
        set_rate_target(0, pitch_rate_degs, yaw_rate_degs, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_FAILED;
}


MAV_RESULT AP_Mount::handle_command_long(const mavlink_command_long_t &packet)
{
    switch (packet.command) {
    case MAV_CMD_DO_MOUNT_CONFIGURE:
        return handle_command_do_mount_configure(packet);
    case MAV_CMD_DO_MOUNT_CONTROL:
        return handle_command_do_mount_control(packet);
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        return handle_command_do_gimbal_manager_pitchyaw(packet);
    default:
        return MAV_RESULT_UNSUPPORTED;
    }
}

/// Change the configuration of the mount
void AP_Mount::handle_global_position_int(const mavlink_message_t &msg)
{
    mavlink_global_position_int_t packet;
    mavlink_msg_global_position_int_decode(&msg, &packet);

    if (!check_latlng(packet.lat, packet.lon)) {
        return;
    }

    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_global_position_int(msg.sysid, packet);
        }
    }
}

/// Change the configuration of the mount
void AP_Mount::handle_mount_configure(const mavlink_message_t &msg)
{
    if (!check_primary()) {
        return;
    }

    mavlink_mount_configure_t packet;
    mavlink_msg_mount_configure_decode(&msg, &packet);

    // send message to backend
    _backends[_primary]->handle_mount_configure(packet);
}

/// Control the mount (depends on the previously set mount configuration)
void AP_Mount::handle_mount_control(const mavlink_message_t &msg)
{
    if (!check_primary()) {
        return;
    }

    mavlink_mount_control_t packet;
    mavlink_msg_mount_control_decode(&msg, &packet);

    // send message to backend
    _backends[_primary]->handle_mount_control(packet);
}

// send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
void AP_Mount::send_gimbal_device_attitude_status(mavlink_channel_t chan)
{
    // call send_gimbal_device_attitude_status for each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_gimbal_device_attitude_status(chan);
        }
    }
}

// run pre-arm check.  returns false on failure and fills in failure_msg
// any failure_msg returned will not include a prefix
bool AP_Mount::pre_arm_checks(char *failure_msg, uint8_t failure_msg_len)
{
    // check type parameters
    for (uint8_t i=0; i<AP_MOUNT_MAX_INSTANCES; i++) {
        if ((state[i]._type != Mount_Type_None) && (_backends[i] == nullptr)) {
            strncpy(failure_msg, "check TYPE", failure_msg_len);
            return false;
        }
    }

    // return true if no mount configured
    if (_num_instances == 0) {
        return true;
    }

    // check healthy
    for (uint8_t i=0; i<AP_MOUNT_MAX_INSTANCES; i++) {
        if ((_backends[i] != nullptr) && !_backends[i]->healthy()) {
            strncpy(failure_msg, "not healthy", failure_msg_len);
            return false;
        }
    }

    return true;
}

// point at system ID sysid
void AP_Mount::set_target_sysid(uint8_t instance, uint8_t sysid)
{
    // call instance's set_roi_cmd
    if (check_instance(instance)) {
        _backends[instance]->set_target_sysid(sysid);
    }
}

// set_roi_target - sets target location that mount should attempt to point towards
void AP_Mount::set_roi_target(uint8_t instance, const Location &target_loc)
{
    // call instance's set_roi_cmd
    if (check_instance(instance)) {
        _backends[instance]->set_roi_target(target_loc);
    }
}

bool AP_Mount::check_primary() const
{
    return check_instance(_primary);
}

bool AP_Mount::check_instance(uint8_t instance) const
{
    return instance < AP_MOUNT_MAX_INSTANCES && _backends[instance] != nullptr;
}

// pass a GIMBAL_REPORT message to the backend
void AP_Mount::handle_gimbal_report(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_gimbal_report(chan, msg);
        }
    }
}

void AP_Mount::handle_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_GIMBAL_REPORT:
        handle_gimbal_report(chan, msg);
        break;
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
        handle_mount_configure(msg);
        break;
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        handle_mount_control(msg);
        break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        handle_global_position_int(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
        handle_gimbal_device_information(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
        handle_gimbal_device_attitude_status(msg);
        break;
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled mount case");
#endif
        break;
    }
}

// handle PARAM_VALUE
void AP_Mount::handle_param_value(const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_param_value(msg);
        }
    }
}


// handle GIMBAL_DEVICE_INFORMATION message
void AP_Mount::handle_gimbal_device_information(const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_gimbal_device_information(msg);
        }
    }
}

// handle GIMBAL_DEVICE_ATTITUDE_STATUS message
void AP_Mount::handle_gimbal_device_attitude_status(const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_gimbal_device_attitude_status(msg);
        }
    }
}

// perform any required parameter conversion
void AP_Mount::convert_params()
{
    // convert JSTICK_SPD to RC_RATE
    if (!_rc_rate_max.configured()) {
        int8_t jstick_spd = 0;
        if (AP_Param::get_param_by_index(this, 16, AP_PARAM_INT8, &jstick_spd) && (jstick_spd > 0)) {
            _rc_rate_max.set_and_save(jstick_spd * 0.3);
        }
    }
}

// singleton instance
AP_Mount *AP_Mount::_singleton;

namespace AP {

AP_Mount *mount()
{
    return AP_Mount::get_singleton();
}

};

#endif /* HAL_MOUNT_ENABLED */
