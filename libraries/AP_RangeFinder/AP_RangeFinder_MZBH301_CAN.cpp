#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_MZBH301_CAN_ENABLED

#include "AP_RangeFinder_MZBH301_CAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>

// update the state of the sensor
void AP_RangeFinder_MZBH301_CAN::update(void)
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
bool AP_RangeFinder_MZBH301_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
	uint16_t dist_cm=0;
    WITH_SEMAPHORE(_sem);
    const uint32_t id = frame.id;

    if (!is_correct_id(get_radar_id(id))) {
        return false;
    }

    const uint16_t target1_dist_cm = (uint16_t)(frame.data[0] << 8U | frame.data[1]);
    const uint16_t target2_dist_cm = (uint16_t)(frame.data[2] << 8U | frame.data[3]);
    const uint16_t target3_dist_cm = (uint16_t)(frame.data[4] << 8U | frame.data[5]);

    dist_cm=MIN(target1_dist_cm,target2_dist_cm);
    dist_cm=MIN(dist_cm,target3_dist_cm);

    accumulate_distance_m(dist_cm * 0.001);

    return true;
}

#endif  // AP_RANGEFINDER_MZBH301_CAN_ENABLED
