#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_NRA24 : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_NRA24(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance, AP_SerialManager &serial_manager);

    // update state
    void update(void);

private:
    // get a reading
    bool reading(uint16_t &cm);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;

    unsigned char buff[14];

    union makeshort {
        unsigned char ch[2];
        unsigned short sh;
    };
};
