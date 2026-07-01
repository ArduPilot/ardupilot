#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_RangeFinder_JIYI_CAN.h"
#include <AP_HAL/utility/sparse-endian.h>

#if AP_RANGEFINDER_JIYI_CAN_ENABLED

/*
  Handle an incoming CAN frame from a JIYI Microbrain radar sensor.

  Both the UAV-R21-1 (obstacle avoidance) and UAV-H30-1 (altitude) models
  share the same 8-byte standard-frame protocol:

    Byte 0–1 : Magic word  0xEA 0x2D  (big-endian uint16 = 0xEA2D)
    Byte 2–3 : Msg type    0x04 0x00  (big-endian uint16 = 0x0400)
    Byte 4–5 : Distance    uint16 big-endian, centimetres
               0x0000 indicates no target detected
    Byte 6–7 : SNR / checksum (not used for ranging)

  Returns true if the frame was a valid JIYI distance frame and was consumed.
*/
bool AP_RangeFinder_JIYI_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);

    // Reject extended frames — JIYI uses standard 11-bit IDs
    if (frame.isExtended()) {
        return false;
    }

    // Verify CAN ID matches the configured sensor address
    const uint16_t id = frame.id & AP_HAL::CANFrame::MaskStdID;
    if (!is_correct_id(id)) {
        return false;
    }

    // Must be exactly 8 bytes
    if (frame.dlc != 8) {
        return false;
    }

    // ---------------------------------------------------------------
    // Byte 0–1: magic word (big-endian)
    // ---------------------------------------------------------------
    const uint16_t magic = be16toh_ptr(&frame.data[0]);
    if (magic != JIYI_MAGIC) {
        return false;
    }

    // ---------------------------------------------------------------
    // Byte 2–3: message type word (big-endian)
    // Only process distance-data frames; silently discard others.
    // ---------------------------------------------------------------
    const uint16_t msg_type = be16toh_ptr(&frame.data[2]);
    if (msg_type != JIYI_MSG_DIST) {
        return true;  // consumed but not a distance frame
    }

    // ---------------------------------------------------------------
    // Byte 4–5: distance in centimetres (big-endian uint16)
    // ---------------------------------------------------------------
    const uint16_t dist_cm = be16toh_ptr(&frame.data[4]);

    if (dist_cm == JIYI_NO_TARGET) {
        // Sensor reports no target — do not update the distance
        return true;
    }

    accumulate_distance_m(dist_cm * 0.01f);

    return true;
}

#endif  // AP_RANGEFINDER_JIYI_CAN_ENABLED
