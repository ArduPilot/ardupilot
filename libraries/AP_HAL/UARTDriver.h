
#ifndef __AP_HAL_UART_DRIVER_H__
#define __AP_HAL_UART_DRIVER_H__

#include <stdint.h>

#include "AP_HAL_Namespace.h"

/* Pure virtual UARTDriver class */

class AP_HAL::UARTDriver {
public:
    UARTDriver() {}
    virtual void init(uint16_t baud) = 0;
    /* XXX stub */
};

#endif // __AP_HAL_UART_DRIVER_H__

