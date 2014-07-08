// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_RANGEFINDER_PULSEDLIGHTLRF_H__
#define __AP_RANGEFINDER_PULSEDLIGHTLRF_H__

#include "RangeFinder.h"

/* Connection diagram
 *
 *        ------------------------------------------------------------------------------------
 *        |           J2-1(LED) J2-2(5V) J2-3(Enable) J2-4(Ref Clk) J2-5(GND) J2-6(GND)      |
 *        |                                                                                  |
 *        |                                                                                  |
 *        |                                      J1-3(I2C Clk) J1-2(I2C Data) J1-1(GND)      |
 *        ------------------------------------------------------------------------------------
 *
 * To connect to APM2.x:
 *    APM I2C Clock <-> J1-3
 *    APM I2C Data  <-> J1-2 
 *    APM GND (from output Rail) <-> J1-1 J2-5
 *    APM 5V (from output Rail fed by ESC or BEC) <-> J2-2
 *
 *  APM2.x's I2C connector from outside edge: GND, Data, CLK, 3.3V
 */

// i2c address
#define AP_RANGEFINDER_PULSEDLIGHTLRF_ADDR   0x42

// min and max distances
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MIN_DISTANCE 0
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MAX_DISTANCE 1400

// registers
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MEASURE_REG           0x00
#define AP_RANGEFINDER_PULSEDLIGHTLRF_DISTHIGH_REG          0x0f    // high byte of distance measurement
#define AP_RANGEFINDER_PULSEDLIGHTLRF_DISTLOW_REG           0x10    // low byte of distance measurement

// command register values
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MSRREG_ACQUIRE        0x04    // Varies based on sensor revision, 0x04 is newest, 0x61 is older

class AP_RangeFinder_PulsedLightLRF : public RangeFinder
{

public:

    // constructor
    AP_RangeFinder_PulsedLightLRF(FilterInt16 *filter);

    // init - simply sets the i2c address
    void init(uint8_t address = AP_RANGEFINDER_PULSEDLIGHTLRF_ADDR);

    // take_reading - ask sensor to make a range reading
    bool            take_reading();

    // read value from sensor and return distance in cm
    int16_t         read();

    // heath
    bool            healthy;

protected:
    uint8_t _addr;
};
#endif  // __AP_RANGEFINDER_PULSEDLIGHTLRF_H__
