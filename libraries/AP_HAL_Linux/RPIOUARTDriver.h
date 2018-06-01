#ifndef __AP_HAL_LINUX_RPIOUARTDRIVER_H__
#define __AP_HAL_LINUX_RPIOUARTDRIVER_H__

#include "AP_HAL_Linux.h"

#include "UARTDriver.h"


class Linux::RPIOUARTDriver : public Linux::UARTDriver {
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

    bool sem_take_nonblocking();
    void sem_give();
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    uint32_t _last_update_timestamp;

    bool _external;

    bool _need_set_baud;
    uint32_t _baudrate;
};

#endif //__AP_HAL_LINUX_RPIOUARTDRIVER_H__
