#pragma once

#include <AP_Mount/AP_Mount.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Camera_SoloGimbal {
public:

#define STATUS_NO_GOPRO             0
#define STATUS_INCOMPATIBLE_GOPRO   1
#define STATUS_GOPRO_CONNECTED      2
#define STATUS_GOPRO_ERROR          3
#define CAPTURE_MODE_VIDEO          0
#define CAPTURE_MODE_PHOTO          1
#define CAPTURE_MODE_BURST          2 //Burst only for Hero 3+
#define CAPTURE_MODE_TIMELAPSE      3
#define CAPTURE_MODE_MULTISHOT      4 //Multishot only for Hero4 

    static void gopro_shutter_toggle();
    static void gopro_capture_mode_toggle();
    static void handle_gopro_heartbeat(mavlink_channel_t chan,
                                       const mavlink_message_t &msg);

private:

    static uint8_t gopro_capture_mode;
    static uint8_t gopro_status;
    static bool gopro_is_recording;
    static mavlink_channel_t heartbeat_channel;
};
