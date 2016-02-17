#pragma once

#include <vector>

#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/utility/OwnPtr.h>

#include "AP_HAL_Linux.h"
#include "Semaphores.h"

#define LINUX_SPI_MAX_BUSES 3

// Fake CS pin to indicate in-kernel handling
#define SPI_CS_KERNEL -1

namespace Linux {

class SPIBus;

class SPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    friend class SPIDeviceManager;
    friend class SPIDevice;

    SPIDeviceDriver(const char *name, uint16_t bus, uint16_t subdev, enum AP_HAL::SPIDeviceType type, uint8_t mode, uint8_t bitsPerWord, int16_t cs_pin, uint32_t lowspeed, uint32_t highspeed);

    void init();
    AP_HAL::Semaphore *get_semaphore();
    bool transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
    void transfer (const uint8_t *data, uint16_t len);
    void set_bus_speed(enum bus_speed speed);

private:
    uint16_t _bus;
    uint16_t _subdev;
    int16_t _cs_pin;
    AP_HAL::DigitalSource *_cs;
    const char *_name;
    uint8_t _mode;
    uint8_t _bitsPerWord;
    uint32_t _lowspeed;
    uint32_t _highspeed;
    uint32_t _speed;
    enum AP_HAL::SPIDeviceType _type;
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _fake_dev;
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    friend class SPIDevice;

    static SPIDeviceManager *from(AP_HAL::SPIDeviceManager *spi_mgr)
    {
        return static_cast<SPIDeviceManager*>(spi_mgr);
    }

    SPIDeviceManager()
    {
        /* Reserve space up-front for 3 buses */
        _buses.reserve(3);
    }

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> get_device(const char *name);

    // Temporary function to interoperate with SPIDeviceDriver interface
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> get_device(SPIDeviceDriver &desc);

    void init();
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDeviceType, uint8_t index = 0);

    static void cs_assert(enum AP_HAL::SPIDeviceType type);
    static void cs_release(enum AP_HAL::SPIDeviceType type);
    static bool transaction(SPIDeviceDriver &driver, const uint8_t *tx, uint8_t *rx, uint16_t len);

protected:
    void _unregister(SPIBus &b);
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _create_device(SPIBus &b, SPIDeviceDriver &device_desc) const;

    std::vector<SPIBus*> _buses;

    static const uint8_t _n_device_desc;
    static SPIDeviceDriver _device[];
};

}
