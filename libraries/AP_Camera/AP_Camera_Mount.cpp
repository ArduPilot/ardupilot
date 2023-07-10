#include "AP_Camera_Mount.h"

#if AP_CAMERA_MOUNT_ENABLED
#include <AP_Mount/AP_Mount.h>

extern const AP_HAL::HAL& hal;

// entry point to actually take a picture.  returns true on success
bool AP_Camera_Mount::trigger_pic()
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        mount->take_picture(0);
        return true;
    }
    return false;
}

// start/stop recording video.  returns true on success
// start_recording should be true to start recording, false to stop recording
bool AP_Camera_Mount::record_video(bool start_recording)
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->record_video(0, start_recording);
    }
    return false;
}

// set zoom specified as a rate or percentage
bool AP_Camera_Mount::set_zoom(ZoomType zoom_type, float zoom_value)
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->set_zoom(0, zoom_type, zoom_value);
    }
    return false;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Camera_Mount::set_focus(FocusType focus_type, float focus_value)
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->set_focus(0, focus_type, focus_value);
    }
    return SetFocusResult::FAILED;
}

// send camera information message to GCS
void AP_Camera_Mount::send_camera_information(mavlink_channel_t chan) const
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->send_camera_information(chan);
    }
}

// send camera settings message to GCS
void AP_Camera_Mount::send_camera_settings(mavlink_channel_t chan) const
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->send_camera_settings(chan);
    }
}

#endif // AP_CAMERA_MOUNT_ENABLED
