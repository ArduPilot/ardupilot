#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_RangeFinder_Benewake_CAN.h"
#include <AP_HAL/utility/sparse-endian.h>

#if AP_RANGEFINDER_BENEWAKE_CAN_ENABLED

const AP_Param::GroupInfo AP_RangeFinder_Benewake_CAN::var_info[] = {

    // @Param: RECV_ID
    // @DisplayName: CAN receive ID
    // @Description: The receive ID of the CAN frames. A value of zero means all IDs are accepted.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("RECV_ID", 10, AP_RangeFinder_Benewake_CAN, receive_id, 0),

    // @Param: SNR_MIN
    // @DisplayName: Minimum signal strength
    // @Description: Minimum signal strength (SNR) to accept distance
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("SNR_MIN", 11, AP_RangeFinder_Benewake_CAN, snr_min, 0),

    AP_GROUPEND
};

Benewake_MultiCAN *AP_RangeFinder_Benewake_CAN::multican;

/*
  constructor
 */
AP_RangeFinder_Benewake_CAN::AP_RangeFinder_Benewake_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
    if (multican == nullptr) {
        multican = new Benewake_MultiCAN();
        if (multican == nullptr) {
            AP_BoardConfig::allocation_error("Benewake_CAN");
        }
    }

    {
        // add to linked list of drivers
        WITH_SEMAPHORE(multican->sem);
        auto *prev = multican->drivers;
        next = prev;
        multican->drivers = this;
    }

    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
}

// update state
void AP_RangeFinder_Benewake_CAN::update(void)
{
    WITH_SEMAPHORE(_sem);
    const uint32_t now = AP_HAL::millis();
    if (_distance_count == 0 && now - state.last_reading_ms > 500) {
        // no new data.
        set_status(RangeFinder::Status::NoData);
    } else if (_distance_count != 0) {
        state.distance_m = 0.01 * (_distance_sum_cm / _distance_count);
        state.last_reading_ms = AP_HAL::millis();
        _distance_sum_cm = 0;
        _distance_count = 0;
        update_status();
    }
}

// handler for incoming frames for H30 radar
bool AP_RangeFinder_Benewake_CAN::handle_frame_H30(AP_HAL::CANFrame &frame)
{
    /*
      The H30 produces 3 targets, each as 16 bit unsigned integers in
      cm. Only look at target1 for now
    */
    const uint16_t target1_cm = be16toh_ptr(&frame.data[0]);
    if (target1_cm == 0) {
        // no target gives 0
        return false;
    }
    //uint16_t target2 = be16toh_ptr(&frame.data[2]);
    //uint16_t target3 = be16toh_ptr(&frame.data[4]);

    _distance_sum_cm += target1_cm;
    _distance_count++;

    return true;
}

// handler for incoming frames. These come in at 100Hz
bool AP_RangeFinder_Benewake_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);
    if (frame.isExtended()) {
        // H30 radar uses extended frames
        const int32_t id = int32_t(frame.id & AP_HAL::CANFrame::MaskExtID);
        if (receive_id != 0 && id != receive_id.get()) {
            // incorrect receive ID
            return false;
        }
        if (last_recv_id != -1 && id != last_recv_id) {
            // changing ID
            return false;
        }
        last_recv_id = id;
        return handle_frame_H30(frame);
    }

    const uint16_t id = frame.id & AP_HAL::CANFrame::MaskStdID;
    if (receive_id != 0 && id != uint16_t(receive_id.get())) {
        // incorrect receive ID
        return false;
    }
    if (last_recv_id != -1 && id != last_recv_id) {
        // changing ID
        return false;
    }
    last_recv_id = id;

    const uint16_t dist_cm = le16toh_ptr(&frame.data[0]);
    const uint16_t snr = le16toh_ptr(&frame.data[2]);
    if (snr_min != 0 && snr < uint16_t(snr_min.get())) {
        // too low signal strength
        return true;
    }
    _distance_sum_cm += dist_cm;
    _distance_count++;
    return true;
}

// handle frames from CANSensor, passing to the drivers
void Benewake_MultiCAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(sem);
    for (auto *d = drivers; d; d=d->next) {
        if (d->handle_frame(frame)) {
            break;
        }
    }
}

#endif  // AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
