
#ifndef __AP_HAL_AVR_UART_DRIVER_H__
#define __AP_HAL_AVR_UART_DRIVER_H__

#include <stdint.h>

#include <AP_HAL.h>

/**
 * AVRUARTDriver is an implementation of UARTDriver for the AVR.
 * It will be a thin wrapper on FastSerial.
 */

class AP_HAL::AVRUARTDriver : public AP_HAL::UARTDriver {
public:
    AVRUARTDriver(int num) : _num(num) {}
    void init(uint16_t baud) { _baud = baud; }
    /* XXX stub implementation! */
private:
    int      _num;
    uint16_t _baud;
};


#endif // __AP_HAL_AVR_UART_DRIVER_H__

