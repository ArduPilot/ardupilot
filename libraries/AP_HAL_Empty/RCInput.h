#pragma once

#include "AP_HAL_Empty.h"

class Empty::RCInput : public AP_HAL::RCInput {
public:
    RCInput();
    void init();
    bool  new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);
};
