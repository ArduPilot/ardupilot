#pragma once

#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Camera_SoloGimbal {
public:

  enum STATUS {
      NO_GOPRO = 0,
      INCOMPATIBLE_GOPRO = 1,
      GOPRO_CONNECTED = 2,
      GOPRO_ERROR = 3
  };

    enum CAPTURE {
        MODE_VIDEO = 0,
        MODE_PHOTO = 1,
        MODE_BURST = 2,   // Burst only for Hero 3+
        MODE_TIMELAPSE = 3,
        MODE_MULTISHOT = 4   // Multishot only for Hero4
    };

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
