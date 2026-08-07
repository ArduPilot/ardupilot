#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BACKEND_CAN_ENABLED

#include "AP_RangeFinder_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

class RangeFinder_MultiCAN;

class AP_RangeFinder_Backend_CAN : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_Backend_CAN(RangeFinder::RangeFinder_State &_state,
                                AP_RangeFinder_Params &_params, AP_CAN::Protocol can_type,
                                const char *driver_name);

    friend class RangeFinder_MultiCAN;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // update state
    virtual void update(void) override;

    // get distance measurement
    bool get_reading(float &reading_m);

    // it is essential that anyone relying on the base-class update to implement this
    virtual bool handle_frame(AP_HAL::CANFrame &frame) = 0;

    // maximum time between readings before we change state to NoData:
    virtual uint32_t read_timeout_ms() const { return 200; }

    // return true if the CAN ID is correct
    bool is_correct_id(uint32_t can_id) const;

    // set distance and count
    void accumulate_distance_m(float distance_m) {
        _distance_sum += distance_m;
        _distance_count++;
    };

    // linked list
    AP_RangeFinder_Backend_CAN *next;

    AP_Int32 receive_id; // CAN ID to receive for this backend
    AP_Int32 snr_min; // minimum signal strength to accept packet

    MultiCAN* multican_rangefinder; // Allows for multiple CAN rangefinders on a single bus
private:

    float _distance_sum; // meters
    uint32_t _distance_count;
};

#endif  // AP_RANGEFINDER_BACKEND_CAN_ENABLED
