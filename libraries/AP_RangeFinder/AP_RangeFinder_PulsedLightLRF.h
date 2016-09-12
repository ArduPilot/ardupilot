// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

/* Connection diagram
 *
 *        ------------------------------------------------------------------------------------
 *        |           J2-1(LED) J2-2(5V) J2-3(Enable) J2-4(Ref Clk) J2-5(GND) J2-6(GND)      |
 *        |                                                                                  |
 *        |                                                                                  |
 *        |                                      J1-3(I2C Clk) J1-2(I2C Data) J1-1(GND)      |
 *        ------------------------------------------------------------------------------------
 */

// i2c address
#define AP_RANGEFINDER_PULSEDLIGHTLRF_ADDR   0x62

// min and max distances
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MIN_DISTANCE 0
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MAX_DISTANCE 1400

// registers
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MEASURE_REG           0x00
#define AP_RANGEFINDER_PULSEDLIGHTLRF_DISTHIGH_REG          0x8f    // high byte of distance measurement

// command register values
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MSRREG_ACQUIRE        0x04    // Varies based on sensor revision, 0x04 is newest, 0x61 is older

class AP_RangeFinder_PulsedLightLRF : public AP_RangeFinder_Backend
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder &ranger, uint8_t instance,
                                          RangeFinder::RangeFinder_State &_state);

    // update state
    void update(void);


private:
    // constructor
    AP_RangeFinder_PulsedLightLRF(RangeFinder &ranger, uint8_t instance,
                                  RangeFinder::RangeFinder_State &_state);

    // start a reading
    bool start_reading(void);
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
