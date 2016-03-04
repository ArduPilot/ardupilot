#pragma once

#include "SerialDevice.h"
#include <AP_HAL/utility/Socket.h>

class ConsoleDevice: public SerialDevice {
public:
    ConsoleDevice();
    virtual ~ConsoleDevice();

    virtual bool open() override;
    virtual bool close() override;
    virtual ssize_t write(const uint8_t *buf, uint16_t n) override;
    virtual ssize_t read(uint8_t *buf, uint16_t n) override;
    virtual void set_blocking(bool blocking) override;
    virtual void set_speed(uint32_t speed) override;

private:
    int _rd_fd = -1;
    int _wr_fd = -1;
    bool _closed = true;
};
