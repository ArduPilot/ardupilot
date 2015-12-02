#ifndef __AP_HAL_LINUX_UARTDEVICE_UDP_H__
#define __AP_HAL_LINUX_UARTDEVICE_UDP_H__

#include "SerialDevice.h"
#include <AP_HAL/utility/Socket.h>

class UARTDevice: public SerialDevice {
public:
    UARTDevice(const char *device_path);
    virtual ~UARTDevice();

    virtual bool open() override;
    virtual bool close() override;
    virtual ssize_t write(const uint8_t *buf, uint16_t n) override;
    virtual ssize_t read(uint8_t *buf, uint16_t n) override;
    virtual void set_blocking(bool blocking) override;
    virtual void set_speed(uint32_t speed) override;

private:
    void _disable_crlf();

    int _fd = -1;
    const char *_device_path;
};

#endif
