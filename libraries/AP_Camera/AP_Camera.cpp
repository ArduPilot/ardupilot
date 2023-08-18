#include "AP_Camera.h"

#if AP_CAMERA_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_GPS/AP_GPS.h>
#include "AP_Camera_Backend.h"
#include "AP_Camera_Servo.h"
#include "AP_Camera_Relay.h"
#include "AP_Camera_SoloGimbal.h"
#include "AP_Camera_Mount.h"
#include "AP_Camera_MAVLink.h"
#include "AP_Camera_MAVLinkCamV2.h"
#include "AP_Camera_Scripting.h"

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

// set camera trigger distance in a mission
void AP_Camera::set_trigger_distance(float distance_m)
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return;
    }
    primary->set_trigger_distance(distance_m);
}

// momentary switch to change camera between picture and video modes
void AP_Camera::cam_mode_toggle()
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return;
    }
    primary->cam_mode_toggle();
}

// take a picture
void AP_Camera::take_picture()
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return;
    }
    primary->take_picture();
}

// take multiple pictures, time_interval between two consecutive pictures is in miliseconds
// total_num is number of pictures to be taken, -1 means capture forever
void AP_Camera::take_multiple_pictures(uint32_t time_interval_ms, int16_t total_num)
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return;
    }
    primary->take_multiple_pictures(time_interval_ms, total_num);
}

// start/stop recording video
// start_recording should be true to start recording, false to stop recording
bool AP_Camera::record_video(bool start_recording)
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return false;
    }
    return primary->record_video(start_recording);
}

// detect and initialise backends
void AP_Camera::init()
{
    // check init has not been called before
    if (primary != nullptr) {
        return;
    }

    // perform any required parameter conversion
    convert_params();

    // create each instance
    for (uint8_t instance = 0; instance < AP_CAMERA_MAX_INSTANCES; instance++) {
        switch ((CameraType)_params[instance].type.get()) {
#if AP_CAMERA_SERVO_ENABLED
        case CameraType::SERVO:
            _backends[instance] = new AP_Camera_Servo(*this, _params[instance], instance);
            break;
#endif
#if AP_CAMERA_RELAY_ENABLED
        case CameraType::RELAY:
            _backends[instance] = new AP_Camera_Relay(*this, _params[instance], instance);
            break;
#endif
#if AP_CAMERA_SOLOGIMBAL_ENABLED
        // check for GoPro in Solo camera
        case CameraType::SOLOGIMBAL:
            _backends[instance] = new AP_Camera_SoloGimbal(*this, _params[instance], instance);
            break;
#endif
#if AP_CAMERA_MOUNT_ENABLED
        // check for Mount camera
        case CameraType::MOUNT:
            _backends[instance] = new AP_Camera_Mount(*this, _params[instance], instance);
            break;
#endif
#if AP_CAMERA_MAVLINK_ENABLED
        // check for MAVLink enabled camera driver
        case CameraType::MAVLINK:
            _backends[instance] = new AP_Camera_MAVLink(*this, _params[instance], instance);
            break;
#endif
#if AP_CAMERA_MAVLINKCAMV2_ENABLED
        // check for MAVLink Camv2 driver
        case CameraType::MAVLINK_CAMV2:
            _backends[instance] = new AP_Camera_MAVLinkCamV2(*this, _params[instance], instance);
            break;
#endif
#if AP_CAMERA_SCRIPTING_ENABLED
        // check for Scripting driver
        case CameraType::SCRIPTING:
            _backends[instance] = new AP_Camera_Scripting(*this, _params[instance], instance);
            break;
#endif
        case CameraType::NONE:
            break;
        }

        // set primary to first non-null instance
        if (primary == nullptr) {
            primary = _backends[instance];
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
    WITH_SEMAPHORE(_rsem);

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

// handle command_long mavlink messages
MAV_RESULT AP_Camera::handle_command_long(const mavlink_command_long_t &packet)
{
    switch (packet.command) {
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
        configure(packet.param1, packet.param2, packet.param3, packet.param4, packet.param5, packet.param6, packet.param7);
        return MAV_RESULT_ACCEPTED;
    case MAV_CMD_DO_DIGICAM_CONTROL:
        control(packet.param1, packet.param2, packet.param3, packet.param4, packet.param5, packet.param6);
        return MAV_RESULT_ACCEPTED;
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        set_trigger_distance(packet.param1);
        if (is_equal(packet.param3, 1.0f)) {
            take_picture();
        }
        return MAV_RESULT_ACCEPTED;
    case MAV_CMD_SET_CAMERA_ZOOM:
        if (is_equal(packet.param1, (float)ZOOM_TYPE_CONTINUOUS) &&
            set_zoom(ZoomType::RATE, packet.param2)) {
            return MAV_RESULT_ACCEPTED;
        }
        if (is_equal(packet.param1, (float)ZOOM_TYPE_RANGE) &&
            set_zoom(ZoomType::PCT, packet.param2)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_UNSUPPORTED;
    case MAV_CMD_SET_CAMERA_FOCUS:
        // accept any of the auto focus types
        switch ((SET_FOCUS_TYPE)packet.param1) {
        case FOCUS_TYPE_AUTO:
        case FOCUS_TYPE_AUTO_SINGLE:
        case FOCUS_TYPE_AUTO_CONTINUOUS:
            return (MAV_RESULT)set_focus(FocusType::AUTO, 0);
        case FOCUS_TYPE_CONTINUOUS:
        // accept continuous manual focus
            return (MAV_RESULT)set_focus(FocusType::RATE, packet.param2);
        // accept focus as percentage
        case FOCUS_TYPE_RANGE:
            return (MAV_RESULT)set_focus(FocusType::PCT, packet.param2);
        case SET_FOCUS_TYPE_ENUM_END:
        case FOCUS_TYPE_STEP:
        case FOCUS_TYPE_METERS:
            // unsupported focus (bad parameter)
            break;
        }
        return MAV_RESULT_DENIED;
    case MAV_CMD_IMAGE_START_CAPTURE:
        if (!is_zero(packet.param2) || !is_equal(packet.param3, 1.0f) || !is_zero(packet.param4)) {
            // Its a multiple picture request
            if (is_equal(packet.param3, 0.0f)) {
                take_multiple_pictures(packet.param2*1000, -1);
            } else {
                take_multiple_pictures(packet.param2*1000, packet.param3);
            }
            return MAV_RESULT_ACCEPTED;
        }
        take_picture();
        return MAV_RESULT_ACCEPTED;
    case MAV_CMD_CAMERA_TRACK_POINT:
        if (set_tracking(TrackingType::TRK_POINT, Vector2f{packet.param1, packet.param2}, Vector2f{})) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_UNSUPPORTED;
    case MAV_CMD_CAMERA_TRACK_RECTANGLE:
        if (set_tracking(TrackingType::TRK_RECTANGLE, Vector2f{packet.param1, packet.param2}, Vector2f{packet.param3, packet.param4})) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_UNSUPPORTED;
    case MAV_CMD_CAMERA_STOP_TRACKING:
        if (set_tracking(TrackingType::TRK_NONE, Vector2f{}, Vector2f{})) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_UNSUPPORTED;
    case MAV_CMD_VIDEO_START_CAPTURE:
    case MAV_CMD_VIDEO_STOP_CAPTURE:
    {
        bool success = false;
        const bool start_recording = (packet.command == MAV_CMD_VIDEO_START_CAPTURE);
        const uint8_t stream_id = packet.param1;  // Stream ID
        if (stream_id == 0) {
            // stream id of 0 interpreted as primary camera
            success = record_video(start_recording);
        } else {
            // convert stream id to instance id
            success = record_video(stream_id - 1, start_recording);
        }
        if (success) {
            return MAV_RESULT_ACCEPTED;
        } else {
            return MAV_RESULT_FAILED;
        }
    }
    default:
        return MAV_RESULT_UNSUPPORTED;
    }
}

// set camera trigger distance in a mission
void AP_Camera::set_trigger_distance(uint8_t instance, float distance_m)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }

    // call backend
    backend->set_trigger_distance(distance_m);
}

// momentary switch to change camera between picture and video modes
void AP_Camera::cam_mode_toggle(uint8_t instance)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }

    // call backend
    backend->cam_mode_toggle();
}

// configure camera
void AP_Camera::configure(float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time)
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return;
    }
    primary->configure(shooting_mode, shutter_speed, aperture, ISO, exposure_type, cmd_id, engine_cutoff_time);
}

void AP_Camera::configure(uint8_t instance, float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }

    // call backend
    backend->configure(shooting_mode, shutter_speed, aperture, ISO, exposure_type, cmd_id, engine_cutoff_time);
}

// handle camera control
void AP_Camera::control(float session, float zoom_pos, float zoom_step, float focus_lock, float shooting_cmd, float cmd_id)
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return;
    }
    primary->control(session, zoom_pos, zoom_step, focus_lock, shooting_cmd, cmd_id);
}

void AP_Camera::control(uint8_t instance, float session, float zoom_pos, float zoom_step, float focus_lock, float shooting_cmd, float cmd_id)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }

    // call backend
    backend->control(session, zoom_pos, zoom_step, focus_lock, shooting_cmd, cmd_id);
}

/*
  Send camera feedback to the GCS
 */
void AP_Camera::send_feedback(mavlink_channel_t chan)
{
    WITH_SEMAPHORE(_rsem);

    // call each instance
    for (uint8_t instance = 0; instance < AP_CAMERA_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_camera_feedback(chan);
        }
    }
}

// send camera information message to GCS
void AP_Camera::send_camera_information(mavlink_channel_t chan)
{
    WITH_SEMAPHORE(_rsem);

    // call each instance
    for (uint8_t instance = 0; instance < AP_CAMERA_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_camera_information(chan);
        }
    }
}

// send camera settings message to GCS
void AP_Camera::send_camera_settings(mavlink_channel_t chan)
{
    WITH_SEMAPHORE(_rsem);

    // call each instance
    for (uint8_t instance = 0; instance < AP_CAMERA_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_camera_settings(chan);
        }
    }
}

/*
  update; triggers by distance moved and camera trigger
*/
void AP_Camera::update()
{
    WITH_SEMAPHORE(_rsem);

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
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }

    // call backend
    backend->take_picture();
}

// start/stop recording video.  returns true on success
// start_recording should be true to start recording, false to stop recording
bool AP_Camera::record_video(uint8_t instance, bool start_recording)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }

    // call backend
    return backend->record_video(start_recording);
}

// zoom specified as a rate or percentage
bool AP_Camera::set_zoom(ZoomType zoom_type, float zoom_value)
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return false;
    }
    return primary->set_zoom(zoom_type, zoom_value);
}

// zoom specified as a rate or percentage
bool AP_Camera::set_zoom(uint8_t instance, ZoomType zoom_type, float zoom_value)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }

    // call each instance
    return backend->set_zoom(zoom_type, zoom_value);
}


// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Camera::set_focus(FocusType focus_type, float focus_value)
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return SetFocusResult::FAILED;
    }
    return primary->set_focus(focus_type, focus_value);
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Camera::set_focus(uint8_t instance, FocusType focus_type, float focus_value)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return SetFocusResult::FAILED;
    }

    // call each instance
    return backend->set_focus(focus_type, focus_value);
}

// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
// p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Camera::set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2)
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return false;
    }
    return primary->set_tracking(tracking_type, p1, p2);
}

// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
// p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Camera::set_tracking(uint8_t instance, TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }

    // call each instance
    return backend->set_tracking(tracking_type, p1, p2);
}

// set camera lens as a value from 0 to 5
bool AP_Camera::set_lens(uint8_t lens)
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return false;
    }
    return primary->set_lens(lens);
}

bool AP_Camera::set_lens(uint8_t instance, uint8_t lens)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }

    // call instance
    return backend->set_lens(lens);
}

#if AP_CAMERA_SCRIPTING_ENABLED
// accessor to allow scripting backend to retrieve state
// returns true on success and cam_state is filled in
bool AP_Camera::get_state(uint8_t instance, camera_state_t& cam_state)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->get_state(cam_state);
}
#endif // #if AP_CAMERA_SCRIPTING_ENABLED

// return backend for instance number
AP_Camera_Backend *AP_Camera::get_instance(uint8_t instance) const
{
    if (instance >= ARRAY_SIZE(_backends)) {
        return nullptr;
    }
    return _backends[instance];
}

// perform any required parameter conversion
void AP_Camera::convert_params()
{
    // exit immediately if CAM1_TYPE has already been configured
    if (_params[0].type.configured()) {
        return;
    }

    // PARAMETER_CONVERSION - Added: Feb-2023 ahead of 4.4 release

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
