#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NRA24_CAN_ENABLED

#include "AP_RangeFinder_NRA24_CAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>

// update the state of the sensor
void AP_RangeFinder_NRA24_CAN::update(void)
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

// handler for incoming frames
bool AP_RangeFinder_NRA24_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);
    const uint32_t id = frame.id;

    if (!is_correct_id(get_radar_id(id))) {
        return false;
    }

    switch (id & 0xFU) {
        case 0xAU:
            // heart beat in the form of Radar Status. The contents of this message aren't really useful so we won't parse them for now
            last_heartbeat_ms = AP_HAL::millis();
            break;

        case 0xCU:
        {
            // Target Information
            const float dist_m = (frame.data[2] * 0x100U + frame.data[3]) * 0.01;
            const uint8_t snr = frame.data[7] - 128;

            if ((snr_min != 0 && snr < uint16_t(snr_min.get()))) {
                // too low signal strength
                return false;
            }
            accumulate_distance_m(dist_m);
        }
            break;

        default:
            // not parsing these messages
            break;
    }

    return true;
}

#endif  // AP_RANGEFINDER_NRA24_CAN_ENABLED
