#pragma once

#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Camera_SoloGimbal {
public:

    static void gopro_shutter_toggle();
    static void gopro_capture_mode_toggle();
    static void handle_gopro_heartbeat(mavlink_channel_t chan, const mavlink_message_t &msg);

private:

    static GOPRO_CAPTURE_MODE gopro_capture_mode;
    static GOPRO_HEARTBEAT_STATUS gopro_status;
    static bool gopro_is_recording;
    static mavlink_channel_t heartbeat_channel;
};
