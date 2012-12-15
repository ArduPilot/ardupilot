
#ifndef __AP_HAL_EMPTY_SPIDRIVER_H__
#define __AP_HAL_EMPTY_SPIDRIVER_H__

#include <AP_HAL_Empty.h>


class Empty::EmptySPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    EmptySPIDeviceDriver();
    void init();
    AP_HAL::Semaphore* get_semaphore();
    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
};

class Empty::EmptySPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    EmptySPIDeviceManager();
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);
private:
    EmptySPIDeviceDriver _device;
};

#endif // __AP_HAL_EMPTY_SPIDRIVER_H__
