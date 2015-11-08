
#ifndef __AP_HAL_LINUX_RCINPUT_BH_H__
#define __AP_HAL_LINUX_RCINPUT_BH_H__

#include "AP_HAL_Linux.h"

#define MAX_GPIOS (8)

class Linux::RCInput_BH : public Linux::RCInput
{
public:
    void init(void*);
    void _timer_tick(void);

private:
    int pigpio;

    struct gpioReport_t
    {
        uint16_t seqno;
        uint16_t flags;
        uint32_t tick;
        uint32_t level;
    } gpioReport;

    uint32_t curtick;
    uint32_t prevtick[MAX_GPIOS];
    uint16_t width_s0[MAX_GPIOS];
    uint16_t width_s1[MAX_GPIOS];
};

#endif // __AP_HAL_LINUX_RCINPUT_BH_H__
