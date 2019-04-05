#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define TRDUO_BUFFER_SIZE_FULL 7
#define TRDUO_VALUE_TO_CM_FACTOR 10
#define TRDUO_MIN_DISTANCE_TOF 20
#define TRDUO_MAX_DISTANCE_TOF 1400
#define TRDUO_MIN_DISTANCE_SOUND 5
#define TRDUO_MAX_DISTANCE_SOUND 765
#define TRDUO_DIFF_LIMIT 30
#define TRDUO_TIMEOUT_MS 300

class AP_RangeFinder_TeraRangerDUO : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_TeraRangerDUO(RangeFinder::RangeFinder_State &_state,
                                 AP_RangeFinder_Params &_params,
                                 AP_SerialManager &serial_manager,
                                 uint8_t serial_instance);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);
    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // check and process replies from sensor
    bool get_reading(uint16_t &distance_cm);
    void update_status();

    // uart driver
    AP_HAL::UARTDriver *uart = nullptr;
    
    uint32_t _last_reading_ms;

    // buffer
    uint8_t _buffer[7];
    uint8_t _buffer_count;
    bool _found_start;
};
