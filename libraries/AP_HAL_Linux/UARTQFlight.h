#pragma once

#include <AP_HAL/AP_HAL.h>
#include "SerialDevice.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

class QFLIGHTDevice: public SerialDevice {
public:
    QFLIGHTDevice(const char *device_path);
    virtual ~QFLIGHTDevice();

    virtual bool open() override;
    virtual bool close() override;
    virtual ssize_t write(const uint8_t *buf, uint16_t n) override;
    virtual ssize_t read(uint8_t *buf, uint16_t n) override;
    virtual void set_blocking(bool blocking) override;
    virtual void set_speed(uint32_t speed) override;

private:
    int32_t fd = -1;
    const char *device_path;
};

#endif
