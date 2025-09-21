#include "AP_ExternalControl.h"

#if AP_EXTERNAL_CONTROL_ENABLED

// singleton instance
AP_ExternalControl *AP_ExternalControl::singleton;

bool AP_ExternalControl::arm(AP_Arming::Method method, bool do_arming_checks)
{
    return AP::arming().arm(method, do_arming_checks);
}

bool AP_ExternalControl::disarm(AP_Arming::Method method, bool do_disarm_checks)
{
    return AP::arming().disarm(method, do_disarm_checks);
}

AP_ExternalControl::AP_ExternalControl()
{
    singleton = this;
}

bool AP_ExternalControl::start_image_capture(uint8_t camera_id, uint32_t interval_msec, uint16_t total_images)
{
#if AP_CAMERA_ENABLED
    auto *camera = AP_Camera::get_singleton();
    if (camera == nullptr) {
        return false;
    }

    if (camera->take_multiple_pictures(camera_id, interval_msec, total_images)) {
        return true;
    }
#endif
    return false;
}

bool AP_ExternalControl::stop_image_capture(uint8_t camera_id)
{
#if AP_CAMERA_ENABLED
    auto *camera = AP_Camera::get_singleton();
    if (camera == nullptr) {
        return false;
    }

    if (camera->stop_capture(camera_id)) {
        return true;
    }
#endif
    return false;
}

uint16_t AP_ExternalControl::total_image_cap(uint8_t camera_id)
{
#if AP_CAMERA_ENABLED
    auto *camera = AP_Camera::get_singleton();
    if (camera == nullptr) {
        return 0;
    }

    return camera->total_image_cap(camera_id);
#else
    return 0;
#endif
}

uint16_t AP_ExternalControl::image_current_seq(uint8_t camera_id)
{
#if AP_CAMERA_ENABLED
    auto *camera = AP_Camera::get_singleton();
    if (camera == nullptr) {
        return 0;
    }

    return camera->image_current_seq(camera_id);
#else
    return 0;
#endif
}

namespace AP
{

AP_ExternalControl *externalcontrol()
{
    return AP_ExternalControl::get_singleton();
}

};

#endif // AP_EXTERNAL_CONTROL_ENABLED
