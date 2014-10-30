
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
    
};

#endif // __AP_HAL_LINUX_RCINPUT_NAVIO_H__
