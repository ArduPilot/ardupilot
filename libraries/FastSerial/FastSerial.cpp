// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-
//
// Interrupt-driven serial transmit/receive library.
//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
// Receive and baudrate calculations derived from the Arduino 
// HardwareSerial driver:
//
//      Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
//
// Transmit algorithm inspired by work:
//
//      Code Jose Julio and Jordi Munoz. DIYDrones.com
//
//      This library is free software; you can redistribute it and/or
//      modify it under the terms of the GNU Lesser General Public
//      License as published by the Free Software Foundation; either
//      version 2.1 of the License, or (at your option) any later version.
//
//      This library is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//      Lesser General Public License for more details.
//
//      You should have received a copy of the GNU Lesser General Public
//      License along with this library; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//


#include "../AP_Common/AP_Common.h"
#include "FastSerial.h"
#include "WProgram.h"

#if defined(__AVR_ATmega1280__)
# define FS_MAX_PORTS   4
#else
# define FS_MAX_PORTS   1
#endif

FastSerial       *__FastSerial__ports[FS_MAX_PORTS];

#define RX_BUFFER_SIZE  sizeof(((FastSerial::RXBuffer *)1)->bytes)
#define TX_BUFFER_SIZE  sizeof(((FastSerial::TXBuffer *)1)->bytes)

// Interrupt handlers //////////////////////////////////////////////////////////

#if 0
#define HANDLERS(_PORT, _RXVECTOR, _TXVECTOR, _UDR)     \
SIGNAL(_RXVECTOR)                                       \
{                                                       \
        unsigned char c = _UDR;                         \
        ports[_PORT]->receive(c);                       \
}                                                       \
                                                        \
SIGNAL(_TXVECTOR)                                       \
{                                                       \
        ports[_PORT]->transmit();                       \
}                                                       \
struct hack

#if defined(__AVR_ATmega8__)
HANDLERS(0, SIG_UART_RECV, SIG_UART_DATA, UDR);
#else
HANDLERS(0, SIG_USART0_RECV, SIG_USART0_DATA, UDR0);
#if defined(__AVR_ATmega1280__)
HANDLERS(1, SIG_USART1_RECV, SIG_USART1_DATA, UDR1);
HANDLERS(2, SIG_USART2_RECV, SIG_USART2_DATA, UDR2);
HANDLERS(3, SIG_USART3_RECV, SIG_USART3_DATA, UDR3);
#endif
#endif
#endif

// Constructor /////////////////////////////////////////////////////////////////

FastSerial::FastSerial(const uint8_t portNumber,
                       volatile uint8_t *ubrrh,
                       volatile uint8_t *ubrrl,
                       volatile uint8_t *ucsra,
                       volatile uint8_t *ucsrb,
                       volatile uint8_t *udr,
                       const uint8_t u2x,
                       const uint8_t portEnableBits,
                       const uint8_t portTxBits)
{
        _ubrrh = ubrrh;
        _ubrrl = ubrrl;
        _ucsra = ucsra;
        _ucsrb = ucsrb;
        _udr   = udr;
        _u2x   = u2x;
        _portEnableBits = portEnableBits;
        _portTxBits     = portTxBits;

        // init buffers
        _txBuffer.head = _txBuffer.tail = 0;
        _rxBuffer.head = _rxBuffer.tail = 0;

        // claim the port
        __FastSerial__ports[portNumber] = this;

        // init stdio
        fdev_setup_stream(&_fd, &FastSerial::_putchar, NULL, _FDEV_SETUP_WRITE);
        fdev_set_udata(&_fd, this);
        if (0 == portNumber)
                stdout = &_fd;          // serial port 0 is always the default console
}

// Public Methods //////////////////////////////////////////////////////////////

void FastSerial::begin(long baud)
{
        uint16_t        baud_setting;
        bool            use_u2x;

        // U2X mode is needed for baud rates higher than (CPU Hz / 16)
        if (baud > F_CPU / 16) {
                use_u2x = true;
        } else {
                // figure out if U2X mode would allow for a better connection
    
                // calculate the percent difference between the baud-rate specified and
                // the real baud rate for both U2X and non-U2X mode (0-255 error percent)
                uint8_t nonu2x_baud_error = abs((int)(255-((F_CPU/(16*(((F_CPU/8/baud-1)/2)+1))*255)/baud)));
                uint8_t u2x_baud_error = abs((int)(255-((F_CPU/(8*(((F_CPU/4/baud-1)/2)+1))*255)/baud)));
    
                // prefer non-U2X mode because it handles clock skew better
                use_u2x = (nonu2x_baud_error > u2x_baud_error);
        }
  
        if (use_u2x) {
                *_ucsra = _BV(_u2x);
                baud_setting = (F_CPU / 4 / baud - 1) / 2;
        } else {
                *_ucsra = 0;
                baud_setting = (F_CPU / 8 / baud - 1) / 2;
        }

        // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
        *_ubrrh = baud_setting >> 8;
        *_ubrrl = baud_setting;

        *_ucsrb |= _portEnableBits;
}

void FastSerial::end()
{
        *_ucsrb &= ~(_portEnableBits | _portTxBits);
}


uint8_t
FastSerial::available(void)
{
        return((RX_BUFFER_SIZE + _rxBuffer.head - _rxBuffer.tail) % RX_BUFFER_SIZE);
}

int
FastSerial::read(void)
{
        uint8_t         c;

        // if the head and tail are equal, the buffer is empty
        if (_rxBuffer.head == _rxBuffer.tail)
                return(-1);

        // pull character from tail
        c = _rxBuffer.bytes[_rxBuffer.tail];
        _rxBuffer.tail = (_rxBuffer.tail + 1) % RX_BUFFER_SIZE;

        return(c);
}

void
FastSerial::flush(void)
{
        // don't reverse this or there may be problems if the RX interrupt
        // occurs after reading the value of _rxBuffer.head but before writing
        // the value to _rxBuffer.tail; the previous value of head
        // may be written to tail, making it appear as if the buffer
        // don't reverse this or there may be problems if the RX interrupt
        // occurs after reading the value of head but before writing
        // the value to tail; the previous value of rx_buffer_head
        // may be written to tail, making it appear as if the buffer
        // were full, not empty.
        _rxBuffer.head = _rxBuffer.tail;

        // don't reverse this or there may be problems if the TX interrupt
        // occurs after reading the value of _txBuffer.tail but before writing
        // the value to _txBuffer.head.
        _txBuffer.tail = _rxBuffer.head;
}

void
FastSerial::write(uint8_t c)
{
        uint8_t         i;

        // wait for room in the tx buffer
        do {
                i = (_txBuffer.head + 1) % RX_BUFFER_SIZE;
        } while (i == _txBuffer.tail);

        // add byte to the buffer
        _txBuffer.bytes[_txBuffer.head] = c;
        _txBuffer.head = i;

        // enable the data-ready interrupt, as it may be off if the buffer is empty
        *_ucsrb |= _portTxBits;
}

void
FastSerial::write(const uint8_t *buffer, int count)
{
        while (count--)
                write(*buffer++);
}

// STDIO emulation /////////////////////////////////////////////////////////////

int
FastSerial::_putchar(char c, FILE *stream)
{
        FastSerial      *fs;

        fs = (FastSerial *)fdev_get_udata(stream);
        fs->write(c);
        return(0);
}

int
FastSerial::_getchar(FILE *stream)
{
        FastSerial      *fs;

        fs = (FastSerial *)fdev_get_udata(stream);

        // We return -1 if there is nothing to read, which the library interprets
        // as an error, which our clients will need to deal with.
        return(fs->read());
}

int
FastSerial::printf(const char *fmt, ...)
{
        va_list ap;
        int     i;

        va_start(ap, fmt);
        i = vfprintf(&_fd, fmt, ap);
        va_end(ap);

        return(i);
}

int
FastSerial::printf_P(const char *fmt, ...)
{
        va_list ap;
        int     i;

        va_start(ap, fmt);
        i = vfprintf_P(stdout, fmt, ap);
        va_end(ap);

        return(i);
}

// Interrupt methods ///////////////////////////////////////////////////////////

void
FastSerial::receive(uint8_t c)
{
        uint8_t         i;

        // if we should be storing the received character into the location
        // just before the tail (meaning that the head would advance to the
        // current location of the tail), we're about to overflow the buffer
        // and so we don't write the character or advance the head.

        i = (_rxBuffer.head + 1) % RX_BUFFER_SIZE;
        if (i != _rxBuffer.tail) {
                _rxBuffer.bytes[_rxBuffer.head] = c;
                _rxBuffer.head = i;
        }
}

void
FastSerial::transmit(void)
{
        // if the buffer is not empty, send the next byte
        if (_txBuffer.head != _txBuffer.tail) {
                *_udr = _txBuffer.bytes[_txBuffer.tail];
                _txBuffer.tail = (_txBuffer.tail + 1) % TX_BUFFER_SIZE;
        }

        // if the buffer is (now) empty, disable the interrupt
        if (_txBuffer.head == _txBuffer.tail)
                *_ucsrb &= ~_portTxBits;
}

