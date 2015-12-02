#ifndef __AP_HAL_LINUX_SERIALDEVICE_H__
#define __AP_HAL_LINUX_SERIALDEVICE_H__

#include <stdint.h>
#include <stdlib.h>

class SerialDevice {
public: 
    virtual ~SerialDevice() {}

    virtual bool open() = 0;
    virtual bool close() = 0;
    virtual ssize_t write(const uint8_t *buf, uint16_t n) = 0;
    virtual ssize_t read(uint8_t *buf, uint16_t n) = 0;
    virtual void set_blocking(bool blocking) = 0;
    virtual void set_speed(uint32_t speed) = 0;
};

#endif
