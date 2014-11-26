
#ifndef __AP_HAL_LINUX_RCINPUT_NAVIO_H__
#define __AP_HAL_LINUX_RCINPUT_NAVIO_H__

#include <AP_HAL_Linux.h>
#include "RCInput.h"

class Linux::LinuxRCInput_Navio : public Linux::LinuxRCInput 
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
    
    int curtick;
    int prevtick;
    uint16_t width_s0;
    uint16_t width_s1;
};

#endif // __AP_HAL_LINUX_RCINPUT_NAVIO_H__
