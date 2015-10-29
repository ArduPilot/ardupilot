
#ifndef __AP_HAL_EMPTY_SPIDRIVER_H__
#define __AP_HAL_EMPTY_SPIDRIVER_H__

#include "AP_HAL_Linux.h"
#include "Semaphores.h"

// Most platforms won't need to declare the spidev bus offset
#ifndef LINUX_SPIDEV_BUS_OFFSET
#define LINUX_SPIDEV_BUS_OFFSET 0
#endif

#define LINUX_SPI_MAX_BUSES 3

// Fake CS pin to indicate in-kernel handling
#define SPI_CS_KERNEL -1

class Linux::SPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    friend class Linux::SPIDeviceManager;
    SPIDeviceDriver(uint16_t bus, uint16_t subdev, enum AP_HAL::SPIDevice type, uint8_t mode, uint8_t bitsPerWord, int16_t cs_pin, uint32_t lowspeed, uint32_t highspeed);
    void init();
    AP_HAL::Semaphore *get_semaphore();
    bool transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
    void transfer (const uint8_t *data, uint16_t len);
    void set_bus_speed(enum bus_speed speed);
    void set_state(State state) override { _state = state; }
    State get_state() override { return _state; }

private:
    uint16_t _bus;
    uint16_t _subdev;
    int16_t _cs_pin;
    AP_HAL::DigitalSource *_cs;
    uint8_t _mode;
    uint8_t _bitsPerWord;
    State _state = State::UNKNOWN;
    uint32_t _lowspeed;
    uint32_t _highspeed;
    uint32_t _speed;
    enum AP_HAL::SPIDevice _type;
    int _fd;	// Per-device FD.
};

class Linux::SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice, uint8_t index = 0);

    static AP_HAL::Semaphore *get_semaphore(uint16_t bus);

    static void cs_assert(enum AP_HAL::SPIDevice type);
    static void cs_release(enum AP_HAL::SPIDevice type);
    static bool transaction(SPIDeviceDriver &driver, const uint8_t *tx, uint8_t *rx, uint16_t len);

private:
    static SPIDeviceDriver _device[];
    static Semaphore _semaphore[LINUX_SPI_MAX_BUSES];
};

#endif // __AP_HAL_LINUX_SPIDRIVER_H__
