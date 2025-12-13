#pragma once

#include <AP_HAL/utility/Socket_native.h>
#include "SerialDevice.h"

class UDPDevice: public SerialDevice {
public:
    UDPDevice(const char *ip, uint16_t port, bool bcast, bool input);
    virtual ~UDPDevice();

    virtual bool open() override;
    virtual bool close() override;
    virtual void set_blocking(bool blocking) override;
    virtual void set_speed(uint32_t speed) override;
    virtual ssize_t write(const uint8_t *buf, uint16_t n) override;
    virtual ssize_t read(uint8_t *buf, uint16_t n) override;
private:
    SocketAPM_native socket{true};
    const char *_ip;
    uint16_t _port;
    bool _bcast;
    bool _input;
    bool _connected = false;
};
