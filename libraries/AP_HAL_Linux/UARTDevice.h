#ifndef __AP_HAL_LINUX_UARTDEVICE_UDP_H__
#define __AP_HAL_LINUX_UARTDEVICE_UDP_H__

#include "SerialDevice.h"
#include "../AP_HAL/utility/Socket.h"

class UARTDevice: public SerialDevice {
public:
    UARTDevice(char *device_path);
    virtual ~UARTDevice();

    virtual bool open() override;
    virtual bool close() override;
    virtual int write(const uint8_t *buf, uint16_t n) override;
    virtual int read(uint8_t *buf, uint16_t n) override;
    virtual void set_nonblocking() override;
    virtual void set_speed(uint32_t speed) override;

private:
    void _disable_crlf();

    int _fd = -1;
    char *_device_path;
};

#endif
