/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <limits.h>
#include <stdarg.h>
#include "Console.h"
#include "vprintf.h"

using namespace AVR_SITL;

SITLConsoleDriver::SITLConsoleDriver() {}

// ConsoleDriver method implementations ///////////////////////////////////////
void SITLConsoleDriver::init(void* base_uart) 
{
    _base_uart = (AP_HAL::UARTDriver*) base_uart;
}


void SITLConsoleDriver::backend_open() 
{
}

void SITLConsoleDriver::backend_close() 
{
}

size_t SITLConsoleDriver::backend_read(uint8_t *data, size_t len) 
{
	return 0;
}

size_t SITLConsoleDriver::backend_write(const uint8_t *data, size_t len) 
{
	return 0;
}

// SITLConsoleDriver private method implementations ////////////////////////////

// BetterStream method implementations /////////////////////////////////////////
void SITLConsoleDriver::print_P(const prog_char_t *s) 
{
        char    c;
        while ('\0' != (c = pgm_read_byte((const prog_char *)s++)))
                write(c);
}

void SITLConsoleDriver::println_P(const prog_char_t *s) 
{
        print_P(s);
        println();
}

void SITLConsoleDriver::printf(const char *fmt, ...) 
{
        va_list ap;
        va_start(ap, fmt);
        vprintf((AP_HAL::Print*)this, 0, fmt, ap);
        va_end(ap);
}

void SITLConsoleDriver::_printf_P(const prog_char *fmt, ...) 
{
        va_list ap;
        va_start(ap, fmt);
        vprintf((AP_HAL::Print*)this, 1, fmt, ap);
        va_end(ap);
}

// Stream method implementations /////////////////////////////////////////
int16_t SITLConsoleDriver::available(void) 
{
    return _base_uart->available();
}

int16_t SITLConsoleDriver::txspace(void) 
{
    return _base_uart->txspace();
}

int16_t SITLConsoleDriver::read() 
{
    return _base_uart->read();
}

int16_t SITLConsoleDriver::peek() 
{
        return _base_uart->peek();
}

// Print method implementations /////////////////////////////////////////

size_t SITLConsoleDriver::write(uint8_t c) 
{
        return _base_uart->write(c);
}


#endif
