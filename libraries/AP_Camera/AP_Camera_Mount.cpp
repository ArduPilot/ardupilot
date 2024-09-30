#include "AP_Camera_Mount.h"

#if AP_CAMERA_MOUNT_ENABLED
#include <AP_Mount/AP_Mount.h>

extern const AP_HAL::HAL& hal;

// entry point to actually take a picture.  returns true on success
bool AP_Camera_Mount::trigger_pic()
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->take_picture(get_mount_instance());
    }
    return false;
}

// start/stop recording video.  returns true on success
// start_recording should be true to start recording, false to stop recording
bool AP_Camera_Mount::record_video(bool start_recording)
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->record_video(get_mount_instance(), start_recording);
    }
    return false;
}

// set zoom specified as a rate or percentage
bool AP_Camera_Mount::set_zoom(ZoomType zoom_type, float zoom_value)
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->set_zoom(get_mount_instance(), zoom_type, zoom_value);
    }
    return false;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Camera_Mount::set_focus(FocusType focus_type, float focus_value)
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->set_focus(get_mount_instance(), focus_type, focus_value);
    }
    return SetFocusResult::FAILED;
}

#if AP_CAMERA_TRACKING_ENABLED
// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only then top_left is the point
// top_left,bottom_right are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Camera_Mount::set_tracking_internal(TrackingType tracking_type, const Vector2f& top_left, const Vector2f& bottom_right)
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->set_tracking(get_mount_instance(), tracking_type, top_left, bottom_right);
    }
    return false;
}
#endif


// set camera lens as a value from 0 to 5
bool AP_Camera_Mount::set_lens(uint8_t lens)
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->set_lens(get_mount_instance(), lens);
    }
    return false;
}

#if HAL_MOUNT_SET_CAMERA_SOURCE_ENABLED
// set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
bool AP_Camera_Mount::set_camera_source(AP_Camera::CameraSource primary_source, AP_Camera::CameraSource secondary_source)
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->set_camera_source(get_mount_instance(), (uint8_t)primary_source, (uint8_t)secondary_source);
    }
    return false;
}
#endif

// send camera information message to GCS
void AP_Camera_Mount::send_camera_information(mavlink_channel_t chan) const
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->send_camera_information(get_mount_instance(), chan);
    }
}

// send camera settings message to GCS
void AP_Camera_Mount::send_camera_settings(mavlink_channel_t chan) const
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->send_camera_settings(get_mount_instance(), chan);
    }
}

// send camera capture status message to GCS
void AP_Camera_Mount::send_camera_capture_status(mavlink_channel_t chan) const
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->send_camera_capture_status(get_mount_instance(), chan);
    }
}

#if AP_CAMERA_SEND_THERMAL_RANGE_ENABLED
// send camera thermal range message to GCS
void AP_Camera_Mount::send_camera_thermal_range(mavlink_channel_t chan) const
{
#if AP_MOUNT_SEND_THERMAL_RANGE_ENABLED
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        mount->send_camera_thermal_range(get_mount_instance(), chan);
    }
#endif
}
#endif // AP_CAMERA_SEND_THERMAL_RANGE_ENABLED

#if AP_CAMERA_SCRIPTING_ENABLED
// change camera settings not normally used by autopilot
bool AP_Camera_Mount::change_setting(CameraSetting setting, float value)
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->change_setting(get_mount_instance(), setting, value);
    }
    return false;
}
#endif

#endif // AP_CAMERA_MOUNT_ENABLED
