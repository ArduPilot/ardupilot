#ifndef __AP_HAL_LINUX_UDPDEVICE_UDP_H__
#define __AP_HAL_LINUX_UDPDEVICE_UDP_H__

#include "SerialDevice.h"
#include "../AP_HAL/utility/Socket.h"

class UDPDevice: public SerialDevice {
public:
    UDPDevice(const char *ip, uint16_t port);
    virtual ~UDPDevice();

    virtual bool open() override;
    virtual bool close() override;
    virtual void set_nonblocking() override;
    virtual void set_speed(uint32_t speed) override;
    virtual int write(const uint8_t *buf, uint16_t n) override;
    virtual int read(uint8_t *buf, uint16_t n) override;
private:
    SocketAPM socket{true};
    const char *_ip;
    uint16_t _port;
};

#endif
