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

// focus in, out or hold.  returns true on success
// focus in = -1, focus hold = 0, focus out = 1
bool AP_Camera_Mount::set_manual_focus_step(int8_t focus_step)
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->set_manual_focus_step(0, focus_step);
    }
    return false;
}

// auto focus.  returns true on success
bool AP_Camera_Mount::set_auto_focus()
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        return mount->set_auto_focus(0);
    }
    return false;
}

#endif // AP_CAMERA_MOUNT_ENABLED
