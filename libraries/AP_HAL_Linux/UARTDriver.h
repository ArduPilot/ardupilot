
#ifndef __AP_HAL_LINUX_UARTDRIVER_H__
#define __AP_HAL_LINUX_UARTDRIVER_H__

#include <AP_HAL_Linux.h>

class Linux::LinuxUARTDriver : public AP_HAL::UARTDriver {
public:
    LinuxUARTDriver();
    /* Linux implementations of UARTDriver virtual methods */
    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    /* Linux implementations of Stream virtual methods */
    int16_t available();
    int16_t txspace();
    int16_t read();

    /* Linux implementations of Print virtual methods */
    size_t write(uint8_t c);

    void set_device_path(const char *path);

private:
    const char *device_path;
    int _fd;
};

#endif // __AP_HAL_LINUX_UARTDRIVER_H__
