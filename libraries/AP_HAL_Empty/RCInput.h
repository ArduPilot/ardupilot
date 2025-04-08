#pragma once

#include "AP_HAL_Empty.h"

class Empty::RCInput : public AP_HAL::RCInput {
public:
    RCInput();
    void init() override;
    bool  new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;
    virtual const char *protocol() const override { return "Empty"; }

};
