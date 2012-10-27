
#include <limits.h>

#include "vprintf.h"

#include <AP_HAL.h>
#include "Console.h"
using namespace AP_HAL_AVR;

AVRConsoleDriver::AVRConsoleDriver() :
    _user_backend(false)
{}

// ConsoleDriver method implementations ///////////////////////////////////////
void AVRConsoleDriver::init(void* base_uart) {
    _base_uart = (AP_HAL::UARTDriver*) base_uart;
}


void AVRConsoleDriver::backend_open() {
    _txbuf.allocate(128);
    _rxbuf.allocate(16);
    _user_backend = true;
}

void AVRConsoleDriver::backend_close() {
    _user_backend = false;
}

int AVRConsoleDriver::backend_read(uint8_t *data, int len) {
    for (int i = 0; i < len; i++) {
        int b = _txbuf.pop();
        if (b != -1) {
            data[i] = (uint8_t) b;
        } else {
            return i;
        }
    }
    return len;
}

int AVRConsoleDriver::backend_write(const uint8_t *data, int len) {
    for (int i = 0; i < len; i++) {
        bool valid = _rxbuf.push(data[i]);
        if (!valid) {
            return i;
        }
    }
    return len;
}

// AVRConsoleDriver private method implementations ////////////////////////////

// BetterStream method implementations /////////////////////////////////////////
void AVRConsoleDriver::print_P(const prog_char_t *s) {
        char    c;
        while ('\0' != (c = pgm_read_byte((const prog_char *)s++)))
                write(c);
}

void AVRConsoleDriver::println_P(const prog_char_t *s) {
        print_P(s);
        println();
}

void AVRConsoleDriver::printf(const char *fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        vprintf((AP_HAL::Print*)this, 0, fmt, ap);
        va_end(ap);
}

void AVRConsoleDriver::_printf_P(const prog_char *fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        vprintf((AP_HAL::Print*)this, 1, fmt, ap);
        va_end(ap);
}

// Stream method implementations /////////////////////////////////////////
int AVRConsoleDriver::available(void) {
    if (_user_backend) {
        return _rxbuf.bytes_used();
    } else {
        return _base_uart->available();
    }
}

int AVRConsoleDriver::txspace(void) {
    if (_user_backend) {
        return _rxbuf.bytes_free();
    } else {
        return _base_uart->txspace();
    }
}

int AVRConsoleDriver::read() {
    if (_user_backend) {
        return _rxbuf.pop();  
    } else {
        return _base_uart->read();
    }
}

int AVRConsoleDriver::peek() {
    if (_user_backend) {
        return _rxbuf.peek();
    } else {
        return _base_uart->peek();
    }
}

// Print method implementations /////////////////////////////////////////

size_t AVRConsoleDriver::write(uint8_t c) {
    if (_user_backend) {
        return (_txbuf.push(c) ? 1 : 0);
    } else {
        return _base_uart->write(c);
    }
}


/**
 * AVRConsoleDriver::Buffer implementation.
 * A synchronous nonblocking ring buffer, based on the AVRUARTDriver::Buffer
 */

bool AVRConsoleDriver::Buffer::allocate(int size) {
    _head = 0;
    _tail = 0;
    uint8_t shift;
    /* Hardcoded max size of 1024. sue me. */
    for ( shift = 1;
          ( 1 << shift ) < 1024 && ( 1 << shift) < size;
          shift++
        ) ;
    uint16_t tmpmask  = (1 << shift) - 1;

    if ( _bytes != NULL ) {
        if ( _mask == tmpmask ) {
            return true;
        }
        free(_bytes);
    }
    _mask = tmpmask;
    _bytes = (uint8_t*) malloc(_mask+1);
    return (_bytes != NULL);
}

bool AVRConsoleDriver::Buffer::push(uint8_t b) {
    uint16_t next = (_head + 1) & _mask;
    if ( next == _tail ) {
        return false;
    }
    _bytes[_head] = b;
    _head = next;
    return true;
}

int AVRConsoleDriver::Buffer::pop() {
    if ( _tail == _head ) {
        return -1;
    }
    uint8_t b = _bytes[_tail];
    _tail = ( _tail + 1 ) & _mask;
    return (int) b;
}

int AVRConsoleDriver::Buffer::peek() {
    if ( _tail == _head ) {
        return -1;
    }
    uint8_t b = _bytes[_tail];
    return (int) b;
}

uint16_t AVRConsoleDriver::Buffer::bytes_used() {
    return ((_head - _tail) & _mask);
}

uint16_t AVRConsoleDriver::Buffer::bytes_free() {
    return ((_mask+1) - ((_head - _tail) & _mask));
}

