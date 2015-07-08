#ifndef __AP_HAL_LINUX_TCPCLIENTDEVICE_H__
#define __AP_HAL_LINUX_TCPCLIENTDEVICE_H__

#include "SerialDevice.h"
#include "../AP_HAL/utility/Socket.h"

class TCPClientDevice: public SerialDevice {
public:
    TCPClientDevice(const char *ip, uint16_t port);
    virtual ~TCPClientDevice();

    virtual bool open() override;
    virtual bool close() override;
    virtual void set_nonblocking() override;
    virtual void set_speed(uint32_t speed) override;
    virtual int write(const uint8_t *buf, uint16_t n) override;
    virtual int read(uint8_t *buf, uint16_t n) override;
private:
    SocketAPM listener{false};
    const char *_ip;
    uint16_t _port;
};

#endif
