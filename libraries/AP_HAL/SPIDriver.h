#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "SPIDevice.h"
#include "utility/OwnPtr.h"

namespace AP_HAL {

class SPIDeviceManager {
public:
    virtual void init() = 0;
    virtual SPIDeviceDriver* device(enum SPIDeviceType, uint8_t index = 0) = 0;
    virtual OwnPtr<SPIDevice> get_device(const char *name)
    {
        return nullptr;
    }
};

/**
 * We still need an abstraction for performing bulk
 * transfers to be portable to other platforms.
 */

class SPIDeviceDriver {
public:
    virtual void init() = 0;
    virtual Semaphore* get_semaphore() = 0;
    virtual bool transaction(const uint8_t *tx, uint8_t *rx, uint16_t len) = 0;

    virtual void cs_assert() = 0;
    virtual void cs_release() = 0;
    virtual uint8_t transfer (uint8_t data) = 0;
    virtual void transfer (const uint8_t *data, uint16_t len) = 0;

    /**
       optional set_bus_speed() interface. This can be used by drivers
       to request higher speed for sensor registers once the sensor is
       initialised. This is used by the MPU6000 driver which can
       handle 20MHz for sensor register reads, but only 1MHz for other
       registers.
     */
    enum bus_speed {
        SPI_SPEED_LOW, SPI_SPEED_HIGH
    };

    virtual void set_bus_speed(enum bus_speed speed) {}
};

}
