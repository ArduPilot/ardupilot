#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_Benewake : public AP_RangeFinder_Backend
{

public:

    enum benewake_model_type {
        BENEWAKE_TF02 = 0,
        BENEWAKE_TFmini = 1
    };

    // constructor
    AP_RangeFinder_Benewake(RangeFinder::RangeFinder_State &_state,
                            AP_RangeFinder_Params &_params,
                            AP_SerialManager &serial_manager,
                            uint8_t serial_instance,
                            benewake_model_type model);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // get a reading
    // distance returned in reading_cm
    bool get_reading(uint16_t &reading_cm);

    AP_HAL::UARTDriver *uart = nullptr;
    benewake_model_type model_type;
    uint8_t linebuf[10];
    uint8_t linebuf_len;
};
