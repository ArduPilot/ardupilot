#pragma once

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Linux.h"
#include "RCInput.h"

#define CHANNELS 8

class Linux::RCInput_UART : public Linux::RCInput
{
public:
    RCInput_UART(const char *path);
    ~RCInput_UART();

    void init() override;
    void _timer_tick(void) override;

private:

    uint8_t _count;
    int8_t _direction;

    int _fd;
    uint8_t *_pdata;
    ssize_t _remain;
    struct PACKED {
        uint16_t magic;
        uint16_t values[CHANNELS];
    } _data;
};
