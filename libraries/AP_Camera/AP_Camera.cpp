#include "AP_Camera.h"

#if AP_CAMERA_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Mount/AP_Mount.h>
#include "AP_Camera_Backend.h"
#include "AP_Camera_Servo.h"
#include "AP_Camera_Relay.h"
#include "AP_Camera_Mount.h"
#include "AP_Camera_MAVLink.h"
#include "AP_Camera_SoloGimbal.h"

const AP_Param::GroupInfo AP_Camera::var_info[] = {

    // @Param: _MAX_ROLL
    // @DisplayName: Maximum photo roll angle.
    // @Description: Postpone shooting if roll is greater than limit. (0=Disable, will shoot regardless of roll).
    // @User: Standard
    // @Units: deg
    // @Range: 0 180
    AP_GROUPINFO("_MAX_ROLL",  7, AP_Camera, _max_roll, 0),

    // @Param: _AUTO_ONLY
    // @DisplayName: Distance-trigging in AUTO mode only
    // @Description: When enabled, trigging by distance is done in AUTO mode only.
    // @Values: 0:Always,1:Only when in AUTO
    // @User: Standard
    AP_GROUPINFO("_AUTO_ONLY",  10, AP_Camera, _auto_mode_only, 0),

    // @Group: 1
    // @Path: AP_Camera_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1", 12, AP_Camera, AP_Camera_Params),

#if AP_CAMERA_MAX_INSTANCES > 1
    // @Group: 2
    // @Path: AP_Camera_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2", 13, AP_Camera, AP_Camera_Params),
#endif

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

AP_Camera::AP_Camera(uint32_t _log_camera_bit) :
    log_camera_bit(_log_camera_bit)
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

// returns camera type of the given instance
AP_Camera::CameraType AP_Camera::get_type(uint8_t instance) const
{
    if (instance >= AP_CAMERA_MAX_INSTANCES) {
        return CameraType::NONE;
    }

    return (CameraType)_params[instance].type.get();
}

// detect and initialise backends
void AP_Camera::init()
{
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    // perform any required parameter conversion
    convert_params();

    // create each instance
    bool primary_set = false;
    for (uint8_t instance = 0; instance < AP_CAMERA_MAX_INSTANCES; instance++) {
        CameraType camera_type = get_type(instance);

        // check for servo camera
        if (camera_type == CameraType::SERVO) {
            _backends[instance] = new AP_Camera_Servo(*this, _params[instance], instance);
            _num_instances++;

        // check for relay camera
        } else if (camera_type == CameraType::RELAY) {
            _backends[instance] = new AP_Camera_Relay(*this, _params[instance], instance);
            _num_instances++;

#if AP_CAMERA_SOLOGIMBAL_ENABLED
        // check for GoPro in Solo camera
        } else if (camera_type == CameraType::SOLOGIMBAL) {
            _backends[instance] = new AP_Camera_SoloGimbal(*this, _params[instance], instance);
            _num_instances++;
#endif

#if AP_CAMERA_MOUNT_ENABLED
        // check for Mount camera
        } else if (camera_type == CameraType::MOUNT) {
            _backends[instance] = new AP_Camera_Mount(*this, _params[instance], instance);
            _num_instances++;
#endif

#if AP_CAMERA_MAVLINK_ENABLED
        // check for MAVLink enabled camera driver
        } else if (camera_type == CameraType::MAVLINK) {
            _backends[instance] = new AP_Camera_MAVLink(*this, _params[instance], instance);
            _num_instances++;
#endif
        }

        // set primary to first non-null index
        if (!primary_set && (_backends[instance] != nullptr)) {
            primary = instance;
            primary_set = true;
        }
    }

    // init each instance, do it after all instances were created, so that they all know things
    for (uint8_t instance = 0; instance < AP_CAMERA_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->init();
        }
    }
}

// handle incoming mavlink messages
void AP_Camera::handle_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_DIGICAM_CONTROL) {
        // decode deprecated MavLink message that controls camera.
        __mavlink_digicam_control_t packet;
        mavlink_msg_digicam_control_decode(&msg, &packet);
        control(packet.session, packet.zoom_pos, packet.zoom_step, packet.focus_lock, packet.shot, packet.command_id);
        return;
    }

    // call each instance
    for (uint8_t instance = 0; instance < AP_CAMERA_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_message(chan, msg);
        }
    }
}

// set camera trigger distance in a mission
void AP_Camera::set_trigger_distance(uint8_t instance, float distance_m)
{
    if (!is_valid_instance(instance)) {
        return;
    }

    // call backend
    _backends[instance]->set_trigger_distance(distance_m);
}

// momentary switch to change camera between picture and video modes
void AP_Camera::cam_mode_toggle(uint8_t instance)
{
    if (!is_valid_instance(instance)) {
        return;
    }

    // call backend
    _backends[instance]->cam_mode_toggle();
}

void AP_Camera::configure(float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time)
{
    configure(primary, shooting_mode, shutter_speed, aperture, ISO, exposure_type, cmd_id, engine_cutoff_time);
}

void AP_Camera::configure(uint8_t instance, float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time)
{
    if (!is_valid_instance(instance)) {
        return;
    }

    // call backend
    _backends[instance]->configure(shooting_mode, shutter_speed, aperture, ISO, exposure_type, cmd_id, engine_cutoff_time);
}

void AP_Camera::control(float session, float zoom_pos, float zoom_step, float focus_lock, float shooting_cmd, float cmd_id)
{
    control(primary, session, zoom_pos, zoom_step, focus_lock, shooting_cmd, cmd_id);
}

void AP_Camera::control(uint8_t instance, float session, float zoom_pos, float zoom_step, float focus_lock, float shooting_cmd, float cmd_id)
{
    if (!is_valid_instance(instance)) {
        return;
    }

    // call backend
    _backends[instance]->control(session, zoom_pos, zoom_step, focus_lock, shooting_cmd, cmd_id);
}

/*
  Send camera feedback to the GCS
 */
void AP_Camera::send_feedback(mavlink_channel_t chan) const
{
    // call each instance
    for (uint8_t instance = 0; instance < AP_CAMERA_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_camera_feedback(chan);
        }
    }
}

/*
  update; triggers by distance moved and camera trigger
*/
void AP_Camera::update()
{
    // call each instance
    for (uint8_t instance = 0; instance < AP_CAMERA_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->update();
        }
    }
}

// take_picture - take a picture
void AP_Camera::take_picture(uint8_t instance)
{
    if (!is_valid_instance(instance)) {
        return;
    }

    // call backend
    _backends[instance]->take_picture();
}

// start/stop recording video.  returns true on success
// start_recording should be true to start recording, false to stop recording
bool AP_Camera::record_video(uint8_t instance, bool start_recording)
{
    if (!is_valid_instance(instance)) {
        return false;
    }

    // call backend
    return _backends[instance]->record_video(start_recording);
}

// zoom in, out or hold.  returns true on success
// zoom out = -1, hold = 0, zoom in = 1
bool AP_Camera::set_zoom_step(uint8_t instance, int8_t zoom_step)
{
    if (!is_valid_instance(instance)) {
        return false;
    }

    // call each instance
    return _backends[instance]->set_zoom_step(zoom_step);
}

// focus in, out or hold.  returns true on success
// focus in = -1, focus hold = 0, focus out = 1
bool AP_Camera::set_manual_focus_step(uint8_t instance, int8_t focus_step)
{
    if (!is_valid_instance(instance)) {
        return false;
    }

    // call backend
    return _backends[instance]->set_manual_focus_step(focus_step);
}

// auto focus.  returns true on success
bool AP_Camera::set_auto_focus(uint8_t instance)
{
    if (!is_valid_instance(instance)) {
        return false;
    }

    // call backend
    return _backends[instance]->set_auto_focus();
}

// check instance number is valid
bool AP_Camera::is_valid_instance(uint8_t instance) const
{
    return ((instance < AP_CAMERA_MAX_INSTANCES) && (_backends[instance] != nullptr));
}

// perform any required parameter conversion
void AP_Camera::convert_params()
{
    // exit immediately if CAM1_TYPE has already been configured
    if (_params[0].type.configured()) {
        return;
    }

    // below conversions added Feb 2023 ahead of 4.4 release

    // convert CAM_TRIGG_TYPE to CAM1_TYPE
    int8_t cam_trigg_type = 0;
    int8_t cam1_type = 0;
    IGNORE_RETURN(AP_Param::get_param_by_index(this, 0, AP_PARAM_INT8, &cam_trigg_type));
    if ((cam_trigg_type == 0) && SRV_Channels::function_assigned(SRV_Channel::k_cam_trigger)) {
        // CAM_TRIGG_TYPE was 0 (Servo) and camera trigger servo function was assigned so set CAM1_TYPE = 1 (Servo)
        cam1_type = 1;
    }
    if ((cam_trigg_type >= 1) && (cam_trigg_type <= 3)) {
        // CAM_TRIGG_TYPE was set to Relay, GoPro or Mount
        cam1_type = cam_trigg_type + 1;
    }
    _params[0].type.set_and_save(cam1_type);

    // convert CAM_DURATION (in deci-seconds) to CAM1_DURATION (in seconds)
    int8_t cam_duration = 0;
    if (AP_Param::get_param_by_index(this, 1, AP_PARAM_INT8, &cam_duration) && (cam_duration > 0)) {
        _params[0].trigger_duration.set_and_save(cam_duration * 0.1);
    }

    // convert CAM_MIN_INTERVAL (in milliseconds) to CAM1__INTRVAL_MIN (in seconds)
    int16_t cam_min_interval = 0;
    if (AP_Param::get_param_by_index(this, 6, AP_PARAM_INT16, &cam_min_interval) && (cam_min_interval > 0)) {
        _params[0].interval_min.set_and_save(cam_min_interval * 0.001f);
    }

    // find Camera's top level key
    uint16_t k_param_camera_key;
    if (!AP_Param::find_top_level_key_by_pointer(this, k_param_camera_key)) {
        return;
    }

    // table parameters to convert without scaling
    static const AP_Param::ConversionInfo camera_param_conversion_info[] {
        { k_param_camera_key, 2, AP_PARAM_INT16, "CAM1_SERVO_ON" },
        { k_param_camera_key, 3, AP_PARAM_INT16, "CAM1_SERVO_OFF" },
        { k_param_camera_key, 4, AP_PARAM_FLOAT, "CAM1_TRIGG_DIST" },
        { k_param_camera_key, 5, AP_PARAM_INT8, "CAM1_RELAY_ON" },
        { k_param_camera_key, 8, AP_PARAM_INT8, "CAM1_FEEDBAK_PIN" },
        { k_param_camera_key, 9, AP_PARAM_INT8, "CAM1_FEEDBAK_POL" },
    };
    uint8_t table_size = ARRAY_SIZE(camera_param_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&camera_param_conversion_info[i], 1.0f);
    }
}

// singleton instance
AP_Camera *AP_Camera::_singleton;

namespace AP {

AP_Camera *camera()
{
    return AP_Camera::get_singleton();
}

}

#endif
