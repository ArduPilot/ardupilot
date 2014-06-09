
#ifndef __AP_HAL_EMPTY_SPIDRIVER_H__
#define __AP_HAL_EMPTY_SPIDRIVER_H__

#include <AP_HAL_Linux.h>
#include "Semaphores.h"

class Linux::LinuxSPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    LinuxSPIDeviceDriver(const char *spipath, uint8_t mode, uint8_t bitsPerWord, uint8_t cs_pin, uint32_t speed);
    void init();
    AP_HAL::Semaphore* get_semaphore();
    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
    void transfer (const uint8_t *data, uint16_t len);
private:
    LinuxSemaphore _semaphore;
    const char *_spipath;
    int _fd;
    uint8_t _cs_pin;
    AP_HAL::DigitalSource *_cs;
    uint8_t _mode;
    uint8_t _bitsPerWord;
    uint32_t _speed;
};

class Linux::LinuxSPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    LinuxSPIDeviceManager();
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);
private:
    LinuxSPIDeviceDriver _device_ms5611;
    LinuxSPIDeviceDriver _device_mpu6000;
    LinuxSPIDeviceDriver _device_mpu9250;
    LinuxSPIDeviceDriver _device_lsm9ds0;
    LinuxSPIDeviceDriver _device_fram;
};

#endif // __AP_HAL_LINUX_SPIDRIVER_H__
