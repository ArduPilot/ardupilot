#pragma once

#include "RangeFinder_Backend.h"

class AP_RangeFinder_Backend_Serial : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_Backend_Serial(RangeFinder::RangeFinder_State &_state,
                                  AP_RangeFinder_Params &_params,
                                  uint8_t serial_instance);

    // static detection function
    static bool detect(uint8_t serial_instance);

protected:

    AP_HAL::UARTDriver *uart = nullptr;
};
