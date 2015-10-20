
#ifndef __AP_HAL_LINUX_RCINPUT_RASPILOT_H__
#define __AP_HAL_LINUX_RCINPUT_RASPILOT_H__

#include "AP_HAL_Linux.h"
#include "RCInput.h"

class Linux::RCInput_Raspilot : public Linux::RCInput
{
public:
    void init(void*);
    
private:
    uint32_t _last_timer;
    
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;
    
    void _poll_data(void);
};

#endif // __AP_HAL_LINUX_RCINPUT_RASPILOT_H__
