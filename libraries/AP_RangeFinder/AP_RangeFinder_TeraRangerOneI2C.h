// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define AP_RANGE_FINDER_TERARANGERI2C_DEFAULT_ADDR   0x30

#define AP_RANGEFINDER_TERARANGERI2C                4
#define AP_RANGE_FINDER_TERARANGERI2C_SCALER        1.0
#define AP_RANGE_FINDER_TERARANGERI2C_MIN_DISTANCE  20
#define AP_RANGE_FINDER_TERARANGERI2C_MAX_DISTANCE  1500

#define AP_RANGE_FINDER_TERARANGERI2C_COMMAND_TAKE_RANGE_READING 0x61
#define AP_RANGE_FINDER_TERARANGERI2C_COMMAND_WHOAMI 0x01

class AP_RangeFinder_TerarangerI2C : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_TerarangerI2C(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance);

    // update state
    void update(void);
    

private:
    // start a reading
    static bool start_reading(void);
    static bool get_reading(uint16_t &reading_cm);
};

uint8_t AP_RangeFinder_TerarangerI2C_crc8(uint8_t *p, uint8_t len);
