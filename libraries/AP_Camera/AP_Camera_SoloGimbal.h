#pragma once

#include "AP_Camera_config.h"

#if AP_CAMERA_SOLOGIMBAL_ENABLED

#include "AP_Camera_Backend.h"
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Camera_SoloGimbal : public AP_Camera_Backend
{
public:

    // Constructor
    using AP_Camera_Backend::AP_Camera_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_SoloGimbal);

    // entry point to actually take a picture.  returns true on success
    bool trigger_pic() override;

    // momentary switch to change camera between picture and video modes
    void cam_mode_toggle() override;

    // handle MAVLink messages from the camera
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg) override;

private:

    GOPRO_CAPTURE_MODE gopro_capture_mode;
    GOPRO_HEARTBEAT_STATUS gopro_status;
    bool gopro_is_recording;
    mavlink_channel_t heartbeat_channel;
};

#endif // AP_CAMERA_SOLOGIMBAL_ENABLED
