#include "SPIDevice.h"
#include "SPIDriver.h"

using namespace Empty;

SPIDeviceDriver::SPIDeviceDriver()
{}

void SPIDeviceDriver::init()
{}

AP_HAL::Semaphore* SPIDeviceDriver::get_semaphore()
{
    return &_semaphore;
}

bool SPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    return true;
}


void SPIDeviceDriver::cs_assert()
{}

void SPIDeviceDriver::cs_release()
{}

uint8_t SPIDeviceDriver::transfer (uint8_t data)
{
    return 0;
}

void SPIDeviceDriver::transfer (const uint8_t *data, uint16_t len)
{
}

SPIDeviceManager::SPIDeviceManager()
{}

void SPIDeviceManager::init()
{}

AP_HAL::SPIDeviceDriver* SPIDeviceManager::device(enum AP_HAL::SPIDeviceType, uint8_t index)
{
    return &_device;
}

AP_HAL::OwnPtr<AP_HAL::SPIDevice>
SPIDeviceManager::get_device(const char *name)
{
    return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(new SPIDevice());
}
