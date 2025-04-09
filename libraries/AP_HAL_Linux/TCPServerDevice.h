#pragma once

#include <AP_HAL/utility/Socket_native.h>
#include "SerialDevice.h"

#ifndef AP_SOCKET_NATIVE_ENABLED
#error "need native"
#endif

class TCPServerDevice: public SerialDevice {
public:
    TCPServerDevice(const char *ip, uint16_t port, bool wait);
    virtual ~TCPServerDevice();

    virtual bool open() override;
    virtual bool close() override;
    virtual void set_blocking(bool blocking) override;
    virtual void set_speed(uint32_t speed) override;
    virtual ssize_t write(const uint8_t *buf, uint16_t n) override;
    virtual ssize_t read(uint8_t *buf, uint16_t n) override;

private:
    SocketAPM_native listener{false};
    SocketAPM_native *sock = nullptr;
    const char *_ip;
    uint16_t _port;
    bool _wait;
    bool _blocking = false;
    uint32_t _last_bind_warning = 0;
};
