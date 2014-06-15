
#ifndef __AP_HAL_MPNG_SPI_DRIVER_H__
#define __AP_HAL_MPNG_SPI_DRIVER_H__

#include <AP_HAL.h>
#include "AP_HAL_MPNG_Namespace.h"
#include "GPIO.h"
#include "SPIDevices.h"
#include "Semaphores.h"

class MPNG::MPNGSPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    void init(void* machtnichts);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice d);

private:
    AVRSPI0DeviceDriver* _dataflash;
};

#endif // __AP_HAL_MPNG_SPI_DRIVER_H__

