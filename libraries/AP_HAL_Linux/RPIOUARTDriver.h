#pragma once

#include "AP_HAL_Linux.h"

#include "UARTDriver.h"
#include <AP_HAL/SPIDevice.h>

namespace Linux {

class RPIOUARTDriver : public UARTDriver {
public:
    RPIOUARTDriver();

    static RPIOUARTDriver *from(AP_HAL::UARTDriver *uart) {
        return static_cast<RPIOUARTDriver*>(uart);
    }

    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void _timer_tick(void);
    bool isExternal(void);

protected:
    int _write_fd(const uint8_t *buf, uint16_t n);
    int _read_fd(uint8_t *buf, uint16_t n);

private:
    bool _in_timer;

    void _bus_timer(void);

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    bool _external;
    bool _registered_callback;

    bool _need_set_baud;
    uint32_t _baudrate;
};

}
