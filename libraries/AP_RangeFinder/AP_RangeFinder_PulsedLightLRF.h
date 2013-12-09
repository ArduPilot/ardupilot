// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_RANGEFINDER_PULSEDLIGHTLRF_H__
#define __AP_RANGEFINDER_PULSEDLIGHTLRF_H__

#include "RangeFinder.h"

/* Connection diagram
 *
 *        ------------------------------------------------------------------------------------
 *        |                 J5-1(NA) J5-2(NA) J5-3(NA) J5-4(NA) J5-5(NA)                     |
 *        |  J1-1(GND)                                                     J2-5(I2C Data)    |
 *        |  J1-2(5V)                                                      J2-4(I2C Clk)     |
 *        |  J1-3(Enable)                                                  J2-3(NA)          |
 *        |  J1-4(Ser RX)                                                  J2-2(Laser 5-20V) |
 *        |  J1-5(Ser TX)                             (HV Bypass)          J2-1(Boost 5V)    |
 *        |                                                                                  |
 *        |     J6-1(NA)  J6-2(NA)                                                           |
 *        ------------------------------------------------------------------------------------
 *
 * To connect to APM2.x:
 *    APM I2C Clock <-> J2-4
 *    APM I2C Data  <-> J2-5 
 *    APM GND (from output Rail) <-> J1-1
 *    APM 5V (from output Rail fed by ESC or BEC) <-> J1-2
 *
 *  APM2.x's I2C connector from outside edge: GND, Data, CLK, 3.3V
 */

// i2c address
#define AP_RANGEFINDER_PULSEDLIGHTLRF_ADDR   0x70

// min and max distances
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MIN_DISTANCE 0
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MAX_DISTANCE 1400

// registers
#define AP_RANGEFINDER_PULSEDLIGHTLRF_COMMAND_REG           0x00
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MODESTATUS_REG        0x01    // mode & status register to turn on/off continuous measurements and averaging
#define AP_RANGEFINDER_PULSEDLIGHTLRF_SIGNALSTRENGTH_REG    0x05    // signal strenght
#define AP_RANGEFINDER_PULSEDLIGHTLRF_VELOCITY_REG          0x06    // velocity sensed
#define AP_RANGEFINDER_PULSEDLIGHTLRF_DISTHIGH_REG          0x07    // high byte of distance measurement
#define AP_RANGEFINDER_PULSEDLIGHTLRF_DISTLOW_REG           0x08    // low byte of distance measurement
#define AP_RANGEFINDER_PULSEDLIGHTLRF_PROCESSINGCNTRL_REG   0x09    // to set criteria for successful reads including min/max distances and signal strengths
#define AP_RANGEFINDER_PULSEDLIGHTLRF_AVERAGING_REG         0x13
#define AP_RANGEFINDER_PULSEDLIGHTLRF_CONTINUOUSRATE_REG    0x14
#define AP_RANGEFINDER_PULSEDLIGHTLRF_AUTOINCREMENT         0x80    // To-Do: this does not work - we still need to read from each registry individually instead of reading from multiple contiguous registries all at once

// command register values
#define AP_RANGEFINDER_PULSEDLIGHTLRF_CMDREG_ACQUISITION    0x01
#define AP_RANGEFINDER_PULSEDLIGHTLRF_CMDREG_READBLOCK      0x02
#define AP_RANGEFINDER_PULSEDLIGHTLRF_CMDREG_READWRITE_RAM  0x03
#define AP_RANGEFINDER_PULSEDLIGHTLRF_CMDREG_ERASEFLASH     0x04
#define AP_RANGEFINDER_PULSEDLIGHTLRF_CMDREG_POWERDOWN      0x06
#define AP_RANGEFINDER_PULSEDLIGHTLRF_CMDREG_READFLASH      0x07
#define AP_RANGEFINDER_PULSEDLIGHTLRF_CMDREG_SOFTRESET      0x09
#define AP_RANGEFINDER_PULSEDLIGHTLRF_CMDREG_STOREUSERCMD   0x0a

// mode & status  - valid values for mode/status register 0x01
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MODESTATUS_CONTINUOUS 0x20
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MODESTATUS_AVERAGING  0x80        

// averaging - valid values for averaging register 0x13
#define AP_RANGEFINDER_PULSEDLIGHTLRF_AVERAGING_NONE        0
#define AP_RANGEFINDER_PULSEDLIGHTLRF_AVERAGING_2           1
#define AP_RANGEFINDER_PULSEDLIGHTLRF_AVERAGING_4           2
#define AP_RANGEFINDER_PULSEDLIGHTLRF_AVERAGING_8           3
#define AP_RANGEFINDER_PULSEDLIGHTLRF_AVERAGING_16          4
#define AP_RANGEFINDER_PULSEDLIGHTLRF_AVERAGING_64          5
#define AP_RANGEFINDER_PULSEDLIGHTLRF_AVERAGING_128         6
#define AP_RANGEFINDER_PULSEDLIGHTLRF_AVERAGING_256         8

// continuous rates - valid values for continuous rate register 0x14
#define AP_RANGEFINDER_PULSEDLIGHTLRF_CONTINUOUSRATE_100_HZ 1
#define AP_RANGEFINDER_PULSEDLIGHTLRF_CONTINUOUSRATE_10_HZ  10
#define AP_RANGEFINDER_PULSEDLIGHTLRF_CONTINUOUSRATE_1_HZ   100

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
    int             read();

    // heath
    bool            healthy;

protected:
    uint8_t _addr;
};
#endif  // __AP_RANGEFINDER_PULSEDLIGHTLRF_H__
