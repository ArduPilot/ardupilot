#pragma once

#include "AP_HAL_QURT.h"
#include <AP_RCProtocol/AP_RCProtocol.h>

#ifndef RC_INPUT_MAX_CHANNELS
#define RC_INPUT_MAX_CHANNELS 18
#endif

class QURT::RCInput : public AP_HAL::RCInput
{
public:
    void init() override;
    bool new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;

    void _timer_tick(void);

private:
    HAL_Semaphore mutex;
    uint16_t values[RC_INPUT_MAX_CHANNELS];
    uint8_t num_chan;
    bool updated;
};
