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
#include <stdio.h>
#include <AP_Math/location.h>
#include <SRV_Channel/SRV_Channel.h>

const AP_Param::GroupInfo AP_Mount::var_info[] = {

    // @Group: 1
    // @Path: AP_Mount_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1", 43, AP_Mount, AP_Mount_Params),

#if AP_MOUNT_MAX_INSTANCES > 1
    // @Group: 2
    // @Path: AP_Mount_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2", 44, AP_Mount, AP_Mount_Params),
#endif

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
            _backends[instance] = new AP_Mount_Servo(*this, _params[instance], true, instance);
            _num_instances++;
#endif

#if HAL_SOLO_GIMBAL_ENABLED
        // check for Solo mounts
        } else if (mount_type == Mount_Type_SoloGimbal) {
            _backends[instance] = new AP_Mount_SoloGimbal(*this, _params[instance], instance);
            _num_instances++;
#endif // HAL_SOLO_GIMBAL_ENABLED

#if HAL_MOUNT_ALEXMOS_ENABLED
        // check for Alexmos mounts
        } else if (mount_type == Mount_Type_Alexmos) {
            _backends[instance] = new AP_Mount_Alexmos(*this, _params[instance], instance);
            _num_instances++;
#endif

#if HAL_MOUNT_STORM32MAVLINK_ENABLED
        // check for SToRM32 mounts using MAVLink protocol
        } else if (mount_type == Mount_Type_SToRM32) {
            _backends[instance] = new AP_Mount_SToRM32(*this, _params[instance], instance);
            _num_instances++;
#endif

#if HAL_MOUNT_STORM32SERIAL_ENABLED
        // check for SToRM32 mounts using serial protocol
        } else if (mount_type == Mount_Type_SToRM32_serial) {
            _backends[instance] = new AP_Mount_SToRM32_serial(*this, _params[instance], instance);
            _num_instances++;
#endif

#if HAL_MOUNT_GREMSY_ENABLED
        // check for Gremsy mounts
        } else if (mount_type == Mount_Type_Gremsy) {
            _backends[instance] = new AP_Mount_Gremsy(*this, _params[instance], instance);
            _num_instances++;
#endif // HAL_MOUNT_GREMSY_ENABLED

#if HAL_MOUNT_SERVO_ENABLED
        // check for BrushlessPWM mounts (uses Servo backend)
        } else if (mount_type == Mount_Type_BrushlessPWM) {
            _backends[instance] = new AP_Mount_Servo(*this, _params[instance], false, instance);
            _num_instances++;
#endif
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

    return (MountType)_params[instance].type.get();
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

// set_mode_to_default - restores the mode to it's default mode held in the MNTx__DEFLT_MODE parameter
//      this operation requires 60us on a Pixhawk/PX4
void AP_Mount::set_mode_to_default(uint8_t instance)
{
    set_mode(instance, (enum MAV_MOUNT_MODE)_params[instance].default_mode.get());
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

    return MAV_RESULT_ACCEPTED;
}


MAV_RESULT AP_Mount::handle_command_do_mount_control(const mavlink_command_long_t &packet)
{
    if (!check_primary()) {
        return MAV_RESULT_FAILED;
    }

    return _backends[_primary]->handle_command_do_mount_control(packet);
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

    // check gimbal device id.  0 is primary, 1 is 1st gimbal, 2 is 2nd gimbal, etc
    if ((packet.param7 < 0) || (packet.param7 > AP_MOUNT_MAX_INSTANCES)) {
        return MAV_RESULT_FAILED;
    }
    const uint8_t gimbal_instance = (packet.param7 < 1) ? get_primary() : (uint8_t)packet.param7 - 1;
    if (!check_instance(gimbal_instance)) {
        return MAV_RESULT_FAILED;
    }

    // param1 : pitch_angle (in degrees)
    // param2 : yaw angle (in degrees)
    const float pitch_angle_deg = packet.param1;
    const float yaw_angle_deg = packet.param2;
    if (!isnan(pitch_angle_deg) && !isnan(yaw_angle_deg)) {
        set_angle_target(gimbal_instance, 0, pitch_angle_deg, yaw_angle_deg, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return MAV_RESULT_ACCEPTED;
    }

    // param3 : pitch_rate (in deg/s)
    // param4 : yaw rate (in deg/s)
    const float pitch_rate_degs = packet.param3;
    const float yaw_rate_degs = packet.param4;
    if (!isnan(pitch_rate_degs) && !isnan(yaw_rate_degs)) {
        set_rate_target(gimbal_instance, 0, pitch_rate_degs, yaw_rate_degs, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
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

// get mount's current attitude in euler angles in degrees.  yaw angle is in body-frame
// returns true on success
bool AP_Mount::get_attitude_euler(uint8_t instance, float& roll_deg, float& pitch_deg, float& yaw_bf_deg)
{
    if (!check_instance(instance)) {
        return false;
    }

    // send command to backend
    return _backends[instance]->get_attitude_euler(roll_deg, pitch_deg, yaw_bf_deg);
}

// run pre-arm check.  returns false on failure and fills in failure_msg
// any failure_msg returned will not include a prefix
bool AP_Mount::pre_arm_checks(char *failure_msg, uint8_t failure_msg_len)
{
    // check type parameters
    for (uint8_t i=0; i<AP_MOUNT_MAX_INSTANCES; i++) {
        if ((_params[i].type != Mount_Type_None) && (_backends[i] == nullptr)) {
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

//
// camera controls for gimbals that include a camera
//

// take a picture.  returns true on success
bool AP_Mount::take_picture(uint8_t instance)
{
    // call instance's take_picture
    if (!check_instance(instance)) {
        return false;
    }
    return _backends[instance]->take_picture();
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount::record_video(uint8_t instance, bool start_recording)
{
    // call instance's record_video
    if (!check_instance(instance)) {
        return false;
    }
    return _backends[instance]->record_video(start_recording);
}

// set camera zoom step.  returns true on success
// zoom out = -1, hold = 0, zoom in = 1
bool AP_Mount::set_zoom_step(uint8_t instance, int8_t zoom_step)
{
    // call instance's set_zoom_step
    if (!check_instance(instance)) {
        return false;
    }
    return _backends[instance]->set_zoom_step(zoom_step);
}

// set focus in, out or hold.  returns true on success
// focus in = -1, focus hold = 0, focus out = 1
bool AP_Mount::set_manual_focus_step(uint8_t instance, int8_t focus_step)
{
    // call instance's set_manual_focus_step
    if (!check_instance(instance)) {
        return false;
    }
    return _backends[instance]->set_manual_focus_step(focus_step);
}

// auto focus.  returns true on success
bool AP_Mount::set_auto_focus(uint8_t instance)
{
    // call instance's set_auto_focus
    if (!check_instance(instance)) {
        return false;
    }
    return _backends[instance]->set_auto_focus();
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
    // exit immediately if MNT1_TYPE has already been configured
    if (_params[0].type.configured()) {
        return;
    }

    // below conversions added Sep 2022 ahead of 4.3 release

    // convert MNT_TYPE to MNT1_TYPE
    int8_t mnt_type = 0;
    IGNORE_RETURN(AP_Param::get_param_by_index(this, 19, AP_PARAM_INT8, &mnt_type));
    if (mnt_type > 0) {
        int8_t stab_roll = 0;
        int8_t stab_pitch = 0;
        IGNORE_RETURN(AP_Param::get_param_by_index(this, 4, AP_PARAM_INT8, &stab_roll));
        IGNORE_RETURN(AP_Param::get_param_by_index(this, 5, AP_PARAM_INT8, &stab_pitch));
        if (mnt_type == 1 && stab_roll == 0 && stab_pitch == 0)  {
            // Servo type without stabilization is changed to BrushlessPWM
            mnt_type = (int8_t)Mount_Type_BrushlessPWM;
        }
    }
    _params[0].type.set_and_save(mnt_type);

    // convert MNT_JSTICK_SPD to MNT1_RC_RATE
    int8_t jstick_spd = 0;
    if (AP_Param::get_param_by_index(this, 16, AP_PARAM_INT8, &jstick_spd) && (jstick_spd > 0)) {
        _params[0].rc_rate_max.set_and_save(jstick_spd * 0.3);
    }

    // find Mount's top level key
    uint16_t k_param_mount_key;
    if (!AP_Param::find_top_level_key_by_pointer(this, k_param_mount_key)) {
        return;
    }

    // table of mount parameters to be converted without scaling
    static const AP_Param::ConversionInfo mnt_param_conversion_info[] {
        { k_param_mount_key, 0, AP_PARAM_INT8, "MNT1_DEFLT_MODE" },
        { k_param_mount_key, 1, AP_PARAM_VECTOR3F, "MNT1_RETRACT" },
        { k_param_mount_key, 2, AP_PARAM_VECTOR3F, "MNT1_NEUTRAL" },
        { k_param_mount_key, 17, AP_PARAM_FLOAT, "MNT1_LEAD_RLL" },
        { k_param_mount_key, 18, AP_PARAM_FLOAT, "MNT1_LEAD_PTCH" },
    };
    uint8_t table_size = ARRAY_SIZE(mnt_param_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&mnt_param_conversion_info[i], 1.0f);
    }

    // mount parameters conversion from centi-degrees to degrees
    static const AP_Param::ConversionInfo mnt_param_deg_conversion_info[] {
        { k_param_mount_key, 8, AP_PARAM_INT16, "MNT1_ROLL_MIN" },
        { k_param_mount_key, 9, AP_PARAM_INT16, "MNT1_ROLL_MAX" },
        { k_param_mount_key, 11, AP_PARAM_INT16, "MNT1_PITCH_MIN" },
        { k_param_mount_key, 12, AP_PARAM_INT16, "MNT1_PITCH_MAX" },
        { k_param_mount_key, 14, AP_PARAM_INT16, "MNT1_YAW_MIN" },
        { k_param_mount_key, 15, AP_PARAM_INT16, "MNT1_YAW_MAX" },
    };
    table_size = ARRAY_SIZE(mnt_param_deg_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&mnt_param_deg_conversion_info[i], 0.01f);
    }

    // struct and array holding mapping between old param table index and new RCx_OPTION value
    struct MountRCConversionTable {
        uint8_t old_rcin_idx;
        uint16_t new_rc_option;
    };
    const struct MountRCConversionTable mnt_rc_conversion_table[] = {
        {7, 212},   // MTN_RC_IN_ROLL to RCx_OPTION = 212 (MOUNT1_ROLL)
        {10, 213},  // MTN_RC_IN_TILT to RCx_OPTION = 213 (MOUNT1_PITCH)
        {13, 214},  // MTN_RC_IN_PAN to RCx_OPTION = 214 (MOUNT1_YAW)
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(mnt_rc_conversion_table); i++) {
        int8_t mnt_rcin = 0;
        if (AP_Param::get_param_by_index(this, mnt_rc_conversion_table[i].old_rcin_idx, AP_PARAM_INT8, &mnt_rcin) && (mnt_rcin > 0)) {
            // get pointers to the appropriate RCx_OPTION parameter
            char pname[17];
            enum ap_var_type ptype;
            snprintf(pname, sizeof(pname), "RC%u_OPTION", (unsigned)mnt_rcin);
            AP_Int16 *rcx_option = (AP_Int16 *)AP_Param::find(pname, &ptype);
            if ((rcx_option != nullptr) && !rcx_option->configured()) {
                rcx_option->set_and_save(mnt_rc_conversion_table[i].new_rc_option);
            }
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
