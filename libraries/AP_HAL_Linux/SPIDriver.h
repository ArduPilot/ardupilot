
#ifndef __AP_HAL_EMPTY_SPIDRIVER_H__
#define __AP_HAL_EMPTY_SPIDRIVER_H__

#include <AP_HAL_Linux.h>
#include "Semaphores.h"

enum LinuxSPIDeviceType {
    LINUX_SPI_DEVICE_MS5611  = 0,
    LINUX_SPI_DEVICE_MPU6000 = 1,
    LINUX_SPI_DEVICE_MPU9250 = 2,
    LINUX_SPI_DEVICE_LSM9DS0 = 3,
    LINUX_SPI_DEVICE_FRAM    = 4,
    LINUX_SPI_DEVICE_NUM_DEVICES = 5
};

#define LINUX_SPI_NUM_BUSES 2

class Linux::LinuxSPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    LinuxSPIDeviceDriver(uint8_t bus, LinuxSPIDeviceType type, uint8_t mode, uint8_t bitsPerWord, uint8_t cs_pin, uint32_t speed);
    void init();
    AP_HAL::Semaphore *get_semaphore();
    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
    void transfer (const uint8_t *data, uint16_t len);

    uint8_t get_bus(void) const { return _bus; }
    AP_HAL::DigitalSource *get_cs(void) const { return _cs; }

private:
    int _fd;
    uint8_t _cs_pin;
    AP_HAL::DigitalSource *_cs;
    uint8_t _mode;
    uint8_t _bitsPerWord;
    uint32_t _speed;
    LinuxSPIDeviceType _type;
    uint8_t _bus;
};

class Linux::LinuxSPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);

    static AP_HAL::Semaphore *get_semaphore(uint8_t bus);

    static void cs_assert(enum LinuxSPIDeviceType type);
    static void cs_release(enum LinuxSPIDeviceType type);

private:
    static LinuxSPIDeviceDriver _device[LINUX_SPI_DEVICE_NUM_DEVICES];
    static LinuxSemaphore _semaphore[LINUX_SPI_NUM_BUSES];
};

#endif // __AP_HAL_LINUX_SPIDRIVER_H__
