
#ifndef __AP_HAL_EMPTY_SPIDRIVER_H__
#define __AP_HAL_EMPTY_SPIDRIVER_H__

#include <AP_HAL_Linux.h>
#include "Semaphores.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE
#define LINUX_SPI_DEVICE_NUM_DEVICES 6
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
#define LINUX_SPI_DEVICE_NUM_DEVICES 2
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#define LINUX_SPI_DEVICE_NUM_DEVICES 2
#else
#define LINUX_SPI_DEVICE_NUM_DEVICES 0
#endif

// Most platforms won't need to declare the spidev bus offset
#ifndef LINUX_SPIDEV_BUS_OFFSET
#define LINUX_SPIDEV_BUS_OFFSET 0
#endif

#define LINUX_SPI_MAX_BUSES 3

// Fake CS pin to indicate in-kernel handling
#define SPI_CS_KERNEL -1

class Linux::LinuxSPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    friend class Linux::LinuxSPIDeviceManager;
    LinuxSPIDeviceDriver(uint16_t bus, uint16_t subdev, enum AP_HAL::SPIDevice type, uint8_t mode, uint8_t bitsPerWord, int16_t cs_pin, uint32_t lowspeed, uint32_t highspeed);
    void init();
    AP_HAL::Semaphore *get_semaphore();
    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

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
    uint8_t _mode;
    uint8_t _bitsPerWord;
    uint32_t _lowspeed;
    uint32_t _highspeed;
    uint32_t _speed;
    enum AP_HAL::SPIDevice _type;
    int _fd;	// Per-device FD.
};

class Linux::LinuxSPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);

    static AP_HAL::Semaphore *get_semaphore(uint16_t bus);

    static void cs_assert(enum AP_HAL::SPIDevice type);
    static void cs_release(enum AP_HAL::SPIDevice type);
    static void transaction(LinuxSPIDeviceDriver &driver, const uint8_t *tx, uint8_t *rx, uint16_t len);

private:
    static LinuxSPIDeviceDriver _device[LINUX_SPI_DEVICE_NUM_DEVICES];
    static LinuxSemaphore _semaphore[LINUX_SPI_MAX_BUSES];
};

#endif // __AP_HAL_LINUX_SPIDRIVER_H__
