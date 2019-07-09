#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

// WASP 200 LRF
// http://www.attolloengineering.com/wasp-200-lrf.html

class AP_RangeFinder_Wasp : public AP_RangeFinder_Backend {

public:
    AP_RangeFinder_Wasp(RangeFinder::RangeFinder_State &_state,
                        AP_RangeFinder_Params &_params,
                        uint8_t serial_instance);

    static bool detect(uint8_t serial_instance);

    void update(void) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    enum wasp_configuration_stage {
        WASP_CFG_RATE,     // set the baudrate
        WASP_CFG_ENCODING, // set the encoding to LBE
        WASP_CFG_PROTOCOL, // set the protocol type used
        WASP_CFG_FRQ,      // set the update frequency
        WASP_CFG_GO,       // start/resume readings
        WASP_CFG_AUT,      // set the auto sensitivity threshold
        WASP_CFG_THR,      // set the sensitivity threshold
        WASP_CFG_MAVG,     // set the moving average filter
        WASP_CFG_MEDF,     // set the median filter windows size
        WASP_CFG_AVG,      // set the multi-pulse averages
        WASP_CFG_AUV,      // enforce auto voltage
        WASP_CFG_DONE      // done configuring
    };

    wasp_configuration_stage configuration_state;

    bool get_reading(uint16_t &reading_cm);

    void parse_response(void);

    AP_HAL::UARTDriver *uart;
    char linebuf[10];
    uint8_t linebuf_len;
    AP_Int16 mavg;
    AP_Int16 medf;
    AP_Int16 frq;
    AP_Int16 avg;
    AP_Int16 thr;
    AP_Int8  baud;
};
