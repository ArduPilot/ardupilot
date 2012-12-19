
#ifndef __AP_HAL_EMPTY_UARTDRIVER_H__
#define __AP_HAL_EMPTY_UARTDRIVER_H__

#include <AP_HAL_Empty.h>

class Empty::EmptyUARTDriver : public AP_HAL::UARTDriver {
public:
    EmptyUARTDriver();
    /* Empty implementations of UARTDriver virtual methods */
    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    /* Empty implementations of BetterStream virtual methods */
    void print_P(const prog_char_t *pstr);
    void println_P(const prog_char_t *pstr);
    void printf(const char *pstr, ...);
    void _printf_P(const prog_char *pstr, ...);

    void vprintf(const char* fmt, va_list ap);
    void vprintf_P(const prog_char* fmt, va_list ap);

    /* Empty implementations of Stream virtual methods */
    int16_t available();
    int16_t txspace();
    int16_t read();
    int16_t peek();

    /* Empty implementations of Print virtual methods */
    size_t write(uint8_t c);
};

#endif // __AP_HAL_EMPTY_UARTDRIVER_H__
