
#ifndef __AP_HAL_UART_DRIVER_H__
#define __AP_HAL_UART_DRIVER_H__

#include <stdint.h>

#include "AP_HAL_Namespace.h"
#include "utility/BetterStream.h"

/* Pure virtual UARTDriver class */
class AP_HAL::UARTDriver : public AP_HAL::BetterStream {
public:
    UARTDriver() {}
    virtual void   begin(long baud) = 0;
    virtual void   begin(long baud,
                     unsigned int rxSpace,
                     unsigned int txSpace) = 0;
    virtual void   end() = 0;
    virtual void   flush() = 0;
};

/* Concrete EmptyUARTDriver class provided for convenience */
class AP_HAL::EmptyUARTDriver : public AP_HAL::UARTDriver {
public:
    EmptyUARTDriver() {}
    /* Empty implementations of UARTDriver virtual methods */
    void begin(long b) {}
    void begin(long b, unsigned int rxS, unsigned int txS) {}
    void end() {}
    void flush() {}

    /* Empty implementations of BetterStream virtual methods */
    void print_P(const prog_char_t *pstr) {}
    void println_P(const prog_char_t *pstr) {}
    void printf(const char *pstr, ...) {}
    void _printf_P(const prog_char_t *pstr, ...) {}
    int txspace() { return 1; }

    /* Empty implementations of Stream virtual methods */
    int available() { return 0; }
    int read() { return -1; }
    int peek() { return -1; }

    /* Empty implementations of Print virtual methods */
    size_t write(uint8_t c) { return 0; }
};


#endif // __AP_HAL_UART_DRIVER_H__

