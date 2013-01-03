
#ifndef __AP_HAL_PX4_UARTDRIVER_H__
#define __AP_HAL_PX4_UARTDRIVER_H__

#include <AP_HAL_PX4.h>

class PX4::PX4UARTDriver : public AP_HAL::UARTDriver {
public:
    PX4UARTDriver();
    /* PX4 implementations of UARTDriver virtual methods */
    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    /* PX4 implementations of BetterStream virtual methods */
    void print_P(const prog_char_t *pstr);
    void println_P(const prog_char_t *pstr);
    void printf(const char *pstr, ...);
    void _printf_P(const prog_char *pstr, ...);

    void vprintf(const char* fmt, va_list ap);
    void vprintf_P(const prog_char* fmt, va_list ap);

    /* PX4 implementations of Stream virtual methods */
    int16_t available();
    int16_t txspace();
    int16_t read();
    int16_t peek();

    /* PX4 implementations of Print virtual methods */
    size_t write(uint8_t c);

    bool _initialised;

private:
    void _vdprintf(int fd, const char *fmt, va_list ap);
    uint16_t _rxBufSize;
    uint16_t _txBufSize;
    int16_t _rxChar;
};

#endif // __AP_HAL_PX4_UARTDRIVER_H__
