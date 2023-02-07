#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#ifndef AP_RANGEFINDER_WASP_ENABLED
#define AP_RANGEFINDER_WASP_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#if AP_RANGEFINDER_WASP_ENABLED

// WASP 200 LRF
// http://www.attolloengineering.com/wasp-200-lrf.html

class AP_RangeFinder_Wasp : public AP_RangeFinder_Backend_Serial {

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_Wasp(_state, _params);
    }

    void update(void) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // Wasp is always 115200
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    AP_RangeFinder_Wasp(RangeFinder::RangeFinder_State &_state,
                        AP_RangeFinder_Params &_params);

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

    wasp_configuration_stage configuration_state = WASP_CFG_PROTOCOL;

    bool get_reading(float &reading_m) override;

    void parse_response(void);

    char linebuf[10];
    uint8_t linebuf_len;
    AP_Int16 mavg;
    AP_Int16 medf;
    AP_Int16 frq;
    AP_Int16 avg;
    AP_Int16 thr;
    AP_Int8  baud;
};

#endif
