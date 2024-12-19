#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_HEXSOON_CAN_ENABLED

#include "AP_RangeFinder_Hexsoon_CAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

// update the state of the sensor
void AP_RangeFinder_Hexsoon_CAN::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (get_reading(state.distance_m)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > read_timeout_ms()) {
        if (AP_HAL::millis() - last_heartbeat_ms > read_timeout_ms()) {
            // no heartbeat, must be disconnected
            set_status(RangeFinder::Status::NotConnected);
        } else {
            // Have heartbeat, just no data. Probably because this sensor doesn't output data when there is no relative motion infront of the radar.
            // This case has special pre-arm check handling
            set_status(RangeFinder::Status::NoData);
        }
    }
}

// handler for incoming frames, returns true if packet consumed
bool AP_RangeFinder_Hexsoon_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);

    // exit immediately if the length is not 8 bytes
    if (frame.dlc != 8) {
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    switch (frame.id) {
        case (uint16_t)MessageId::PARAMETER_CONFIGURATION:
        case (uint16_t)MessageId::RADAR_STATUS_INFO:
        case (uint16_t)MessageId::OBJECT_LIST_STATUS:
        case (uint16_t)MessageId::VERSION_INFO:
            // ignore but use for health monitoring
            last_heartbeat_ms = now_ms;
            break;

        case (uint16_t)MessageId::OBJECT_GENERAL_INFO:
        {
            // distance and relative velocity information for a single tracked object
            // only process middle sector
            const uint8_t sector = (frame.data[6] >> 3) & 0x03;
            if (sector != 1) {
                return true;
            }

            // check signal strength
            const float receiver_strength = (frame.data[7] * 0.5) - 64;
            if ((snr_min != 0 && receiver_strength < uint16_t(snr_min.get()))) {
                // too low signal strength
                return true;
            }

            // calculate distance
            const float dist_vert = ((int16_t(frame.data[1]) << 5) + (int16_t(frame.data[2] >> 3))) * 0.2 - 500;
            const float dist_horiz = ((int16_t(frame.data[2] & 0x07) << 8) + frame.data[3]) * 0.2 - 204.6;
            const float dist_m = sqrtF(sq(dist_vert) + sq(dist_horiz));
            accumulate_distance_m(dist_m);
            break;
        }

        default:
            // not parsing these messages
            return false;
    }

    return true;
}

#endif  // AP_RANGEFINDER_HEXSOON_CAN_ENABLED
