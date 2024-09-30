#include "AP_Camera.h"

#if AP_CAMERA_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
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
bool AP_Camera::take_picture()
{
    WITH_SEMAPHORE(_rsem);

    // call for each instance
    bool success = false;
    for (uint8_t i = 0; i < AP_CAMERA_MAX_INSTANCES; i++) {
        if (_backends[i] != nullptr) {
            success |= _backends[i]->take_picture();
        }
    }

    // return true if at least once pic taken
    return success;
}

bool AP_Camera::take_picture(uint8_t instance)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->take_picture();
}

// take multiple pictures, time_interval between two consecutive pictures is in miliseconds
// if instance is not provided, all available cameras affected
// time_interval_ms must be positive
// total_num is number of pictures to be taken, -1 means capture forever
// returns true if at least one camera is successful
bool AP_Camera::take_multiple_pictures(uint32_t time_interval_ms, int16_t total_num)
{
    WITH_SEMAPHORE(_rsem);

    // sanity check time interval
    if (time_interval_ms == 0) {
        return false;
    }

    // call for all instances
    bool success = false;
    for (uint8_t i = 0; i < AP_CAMERA_MAX_INSTANCES; i++) {
        if (_backends[i] != nullptr) {
            _backends[i]->take_multiple_pictures(time_interval_ms, total_num);
            success = true;
        }
    }

    // return true if at least once backend was successful
    return success;
}

bool AP_Camera::take_multiple_pictures(uint8_t instance, uint32_t time_interval_ms, int16_t total_num)
{
    WITH_SEMAPHORE(_rsem);

    // sanity check time interval
    if (time_interval_ms == 0) {
        return false;
    }

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    backend->take_multiple_pictures(time_interval_ms, total_num);
    return true;
}

// stop capturing multiple image sequence
void AP_Camera::stop_capture()
{
    WITH_SEMAPHORE(_rsem);

    // call for each instance
    for (uint8_t i = 0; i < AP_CAMERA_MAX_INSTANCES; i++) {
        if (_backends[i] != nullptr) {
            _backends[i]->stop_capture();
        }
    }
}

bool AP_Camera::stop_capture(uint8_t instance)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    backend->stop_capture();
    return true;
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
            _backends[instance] = NEW_NOTHROW AP_Camera_Servo(*this, _params[instance], instance);
            break;
#endif
#if AP_CAMERA_RELAY_ENABLED
        case CameraType::RELAY:
            _backends[instance] = NEW_NOTHROW AP_Camera_Relay(*this, _params[instance], instance);
            break;
#endif
#if AP_CAMERA_SOLOGIMBAL_ENABLED
        // check for GoPro in Solo camera
        case CameraType::SOLOGIMBAL:
            _backends[instance] = NEW_NOTHROW AP_Camera_SoloGimbal(*this, _params[instance], instance);
            break;
#endif
#if AP_CAMERA_MOUNT_ENABLED
        // check for Mount camera
        case CameraType::MOUNT:
            _backends[instance] = NEW_NOTHROW AP_Camera_Mount(*this, _params[instance], instance);
            break;
#endif
#if AP_CAMERA_MAVLINK_ENABLED
        // check for MAVLink enabled camera driver
        case CameraType::MAVLINK:
            _backends[instance] = NEW_NOTHROW AP_Camera_MAVLink(*this, _params[instance], instance);
            break;
#endif
#if AP_CAMERA_MAVLINKCAMV2_ENABLED
        // check for MAVLink Camv2 driver
        case CameraType::MAVLINK_CAMV2:
            _backends[instance] = NEW_NOTHROW AP_Camera_MAVLinkCamV2(*this, _params[instance], instance);
            break;
#endif
#if AP_CAMERA_SCRIPTING_ENABLED
        // check for Scripting driver
        case CameraType::SCRIPTING:
            _backends[instance] = NEW_NOTHROW AP_Camera_Scripting(*this, _params[instance], instance);
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
MAV_RESULT AP_Camera::handle_command(const mavlink_command_int_t &packet)
{
    switch (packet.command) {
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
        configure(packet.param1, packet.param2, packet.param3, packet.param4, packet.x, packet.y, packet.z);
        return MAV_RESULT_ACCEPTED;
    case MAV_CMD_DO_DIGICAM_CONTROL:
        control(packet.param1, packet.param2, packet.param3, packet.param4, packet.x, packet.y);
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

#if AP_CAMERA_SET_CAMERA_SOURCE_ENABLED
    case MAV_CMD_SET_CAMERA_SOURCE:
        // sanity check instance
        if (is_negative(packet.param1) || packet.param1 > AP_CAMERA_MAX_INSTANCES) {
            return MAV_RESULT_DENIED;
        }
        if (is_zero(packet.param1)) {
            // set camera source for all backends
            bool accepted = false;
            for (uint8_t i = 0; i < ARRAY_SIZE(_backends); i++) {
                if (_backends[i] != nullptr) {
                    accepted |= set_camera_source(i, (AP_Camera::CameraSource)packet.param2, (AP_Camera::CameraSource)packet.param3);
                }
            }
            return accepted ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED;
        }
        if (set_camera_source(packet.param1-1, (AP_Camera::CameraSource)packet.param2, (AP_Camera::CameraSource)packet.param3)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_DENIED;
#endif

    case MAV_CMD_IMAGE_START_CAPTURE:
        // param1 : camera id
        // param2 : interval (in seconds)
        // param3 : total num images
        // sanity check instance
        if (is_negative(packet.param1)) {
            return MAV_RESULT_UNSUPPORTED;
        }
        // check if this is a single picture request (e.g. total images is 1 or interval and total images are zero)
        if (is_equal(packet.param3, 1.0f) ||
            (is_zero(packet.param2) && is_zero(packet.param3))) {
            if (is_zero(packet.param1)) {
                // take pictures for every backend
                return take_picture() ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
            }
            // take picture for specified instance
            return take_picture(packet.param1-1) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
        } else if (is_zero(packet.param3)) {
            // multiple picture request, take pictures forever
            if (is_zero(packet.param1)) {
                // take pictures for every backend
                return take_multiple_pictures(packet.param2*1000, -1) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
            }
            return take_multiple_pictures(packet.param1-1, packet.param2*1000, -1) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
        } else {
            // take multiple pictures equal to the number specified in param3
            if (is_zero(packet.param1)) {
                // take pictures for every backend
                return take_multiple_pictures(packet.param2*1000, packet.param3) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
            }
            return take_multiple_pictures(packet.param1-1, packet.param2*1000, packet.param3) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
        }
    case MAV_CMD_IMAGE_STOP_CAPTURE:
        // param1 : camera id
        if (is_negative(packet.param1)) {
            return MAV_RESULT_UNSUPPORTED;
        }
        if (is_zero(packet.param1)) {
            // stop capture for every backend
            stop_capture();
            return MAV_RESULT_ACCEPTED;
        }
        if (stop_capture(packet.param1-1)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_UNSUPPORTED;
    case MAV_CMD_CAMERA_TRACK_POINT:
#if AP_CAMERA_TRACKING_ENABLED
        if (set_tracking(TrackingType::TRK_POINT, Vector2f{packet.param1, packet.param2}, Vector2f{})) {
            return MAV_RESULT_ACCEPTED;
        }
#endif
        return MAV_RESULT_UNSUPPORTED;
    case MAV_CMD_CAMERA_TRACK_RECTANGLE:
#if AP_CAMERA_TRACKING_ENABLED
        if (set_tracking(TrackingType::TRK_RECTANGLE, Vector2f{packet.param1, packet.param2}, Vector2f{packet.param3, packet.param4})) {
            return MAV_RESULT_ACCEPTED;
        }
#endif
        return MAV_RESULT_UNSUPPORTED;
    case MAV_CMD_CAMERA_STOP_TRACKING:
#if AP_CAMERA_TRACKING_ENABLED
        if (set_tracking(TrackingType::TRK_NONE, Vector2f{}, Vector2f{})) {
            return MAV_RESULT_ACCEPTED;
        }
#endif
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

// send a mavlink message; returns false if there was not space to
// send the message, true otherwise
bool AP_Camera::send_mavlink_message(GCS_MAVLINK &link, const enum ap_message msg_id)
{
    const auto chan = link.get_chan();

    switch (msg_id) {
    case MSG_CAMERA_FEEDBACK:
        CHECK_PAYLOAD_SIZE2(CAMERA_FEEDBACK);
        send_feedback(chan);
        break;
    case MSG_CAMERA_INFORMATION:
        CHECK_PAYLOAD_SIZE2(CAMERA_INFORMATION);
        send_camera_information(chan);
        break;
    case MSG_CAMERA_SETTINGS:
        CHECK_PAYLOAD_SIZE2(CAMERA_SETTINGS);
        send_camera_settings(chan);
        break;
#if AP_CAMERA_SEND_FOV_STATUS_ENABLED
    case MSG_CAMERA_FOV_STATUS:
        CHECK_PAYLOAD_SIZE2(CAMERA_FOV_STATUS);
        send_camera_fov_status(chan);
        break;
#endif
    case MSG_CAMERA_CAPTURE_STATUS:
        CHECK_PAYLOAD_SIZE2(CAMERA_CAPTURE_STATUS);
        send_camera_capture_status(chan);
        break;
#if AP_CAMERA_SEND_THERMAL_RANGE_ENABLED
    case MSG_CAMERA_THERMAL_RANGE:
        CHECK_PAYLOAD_SIZE2(CAMERA_THERMAL_RANGE);
        send_camera_thermal_range(chan);
        break;
#endif

    default:
        // should not reach this; should only be called for specific IDs
        break;
    }

    return true;
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
void AP_Camera::configure(float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time)
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return;
    }
    primary->configure(shooting_mode, shutter_speed, aperture, ISO, exposure_type, cmd_id, engine_cutoff_time);
}

void AP_Camera::configure(uint8_t instance, float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time)
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
void AP_Camera::control(float session, float zoom_pos, float zoom_step, float focus_lock, int32_t shooting_cmd, int32_t cmd_id)
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return;
    }
    primary->control(session, zoom_pos, zoom_step, focus_lock, shooting_cmd, cmd_id);
}

void AP_Camera::control(uint8_t instance, float session, float zoom_pos, float zoom_step, float focus_lock, int32_t shooting_cmd, int32_t cmd_id)
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

#if AP_CAMERA_SEND_FOV_STATUS_ENABLED
// send camera field of view status
void AP_Camera::send_camera_fov_status(mavlink_channel_t chan)
{
    WITH_SEMAPHORE(_rsem);

    // call each instance
    for (uint8_t instance = 0; instance < AP_CAMERA_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_camera_fov_status(chan);
        }
    }
}
#endif

// send camera capture status message to GCS
void AP_Camera::send_camera_capture_status(mavlink_channel_t chan)
{
    WITH_SEMAPHORE(_rsem);

    // call each instance
    for (uint8_t instance = 0; instance < AP_CAMERA_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_camera_capture_status(chan);
        }
    }
}

#if AP_CAMERA_SEND_THERMAL_RANGE_ENABLED
// send camera thermal range message to GCS
void AP_Camera::send_camera_thermal_range(mavlink_channel_t chan)
{
    WITH_SEMAPHORE(_rsem);

    // call each instance
    for (uint8_t instance = 0; instance < AP_CAMERA_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_camera_thermal_range(chan);
        }
    }
}
#endif

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

#if AP_CAMERA_TRACKING_ENABLED
// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only then top_left is the point
// top_left,bottom_right are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Camera::set_tracking(TrackingType tracking_type, const Vector2f& top_left, const Vector2f& bottom_right)
{
    WITH_SEMAPHORE(_rsem);

    if (primary == nullptr) {
        return false;
    }
    return primary->set_tracking(tracking_type, top_left, bottom_right);
}

// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only then top_left is the point
// top_left,bottom_right are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Camera::set_tracking(uint8_t instance, TrackingType tracking_type, const Vector2f& top_left, const Vector2f& bottom_right)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }

    // call each instance
    return backend->set_tracking(tracking_type, top_left, bottom_right);
}
#endif

#if AP_CAMERA_SET_CAMERA_SOURCE_ENABLED
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

// set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
bool AP_Camera::set_camera_source(uint8_t instance, CameraSource primary_source, CameraSource secondary_source)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }

    // call instance
    return backend->set_camera_source(primary_source, secondary_source);
}
#endif // AP_CAMERA_SET_CAMERA_SOURCE_ENABLED

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

// change camera settings not normally used by autopilot
bool AP_Camera::change_setting(uint8_t instance, CameraSetting setting, float value)
{
    WITH_SEMAPHORE(_rsem);

    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->change_setting(setting, value);
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

#if AP_RELAY_ENABLED
// Return true and the relay index if relay camera backend is selected, used for conversion to relay functions
bool AP_Camera::get_legacy_relay_index(int8_t &index) const
{
    // PARAMETER_CONVERSION - Added: Dec-2023

    // Note that this assumes that the camera param conversion has already been done
    // Copter, Plane, Sub and Rover all have both relay and camera and all init relay first
    // This will only be a issue if the relay and camera conversion were done at once, if the user skipped 4.4
    for (uint8_t i = 0; i < AP_CAMERA_MAX_INSTANCES; i++) {
#if AP_CAMERA_RELAY_ENABLED
        if ((CameraType)_params[i].type.get() == CameraType::RELAY) {
            // Camera was hard coded to relay 0
            index = 0;
            return true;
        }
#endif
    }
    return false;
}
#endif

// singleton instance
AP_Camera *AP_Camera::_singleton;

namespace AP {

AP_Camera *camera()
{
    return AP_Camera::get_singleton();
}

}

#endif
