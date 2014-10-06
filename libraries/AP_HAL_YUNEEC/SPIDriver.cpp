#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "SPIDriver.h"

using namespace YUNEEC;

YUNEECSPIDeviceDriver::YUNEECSPIDeviceDriver()
{}

void YUNEECSPIDeviceDriver::init()
{}

AP_HAL::Semaphore* YUNEECSPIDeviceDriver::get_semaphore()
{
    return &_semaphore;
}

void YUNEECSPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{}


void YUNEECSPIDeviceDriver::cs_assert()
{}

void YUNEECSPIDeviceDriver::cs_release()
{}

uint8_t YUNEECSPIDeviceDriver::transfer (uint8_t data)
{
    return 0;
}

void YUNEECSPIDeviceDriver::transfer (const uint8_t *data, uint16_t len)
{
}

YUNEECSPIDeviceManager::YUNEECSPIDeviceManager()
{}

void YUNEECSPIDeviceManager::init(void *)
{}

AP_HAL::SPIDeviceDriver* YUNEECSPIDeviceManager::device(enum AP_HAL::SPIDevice)
{
    return &_device;
}

#endif
