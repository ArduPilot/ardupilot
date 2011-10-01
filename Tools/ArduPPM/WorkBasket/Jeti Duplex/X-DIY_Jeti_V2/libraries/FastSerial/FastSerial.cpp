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


//#include "../AP_Common/AP_Common.h"
#include "FastSerial.h"
#include "WProgram.h"

#if defined(__AVR_ATmega1280__)
# define FS_MAX_PORTS   4
#else
# define FS_MAX_PORTS   1
#endif

FastSerial::Buffer	__FastSerial__rxBuffer[FS_MAX_PORTS];
FastSerial::Buffer	__FastSerial__txBuffer[FS_MAX_PORTS];

// Default buffer sizes
#define RX_BUFFER_SIZE  128
#define TX_BUFFER_SIZE  64
#define BUFFER_MAX      512

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
//HANDLERS(3, SIG_USART3_RECV, SIG_USART3_DATA, UDR3);
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
        _rxBuffer = &__FastSerial__rxBuffer[portNumber];
        _txBuffer->head = _txBuffer->tail = 0;
        _txBuffer = &__FastSerial__txBuffer[portNumber];
        _rxBuffer->head = _rxBuffer->tail = 0;

        // init stdio
        fdev_setup_stream(&_fd, &FastSerial::_putchar, &FastSerial::_getchar, _FDEV_SETUP_RW);
        fdev_set_udata(&_fd, this);
        if (0 == portNumber) {
                stdout = &_fd;          // serial port 0 is always the default console
                stdin = &_fd;
                stderr = &_fd;
        }
}

// Public Methods //////////////////////////////////////////////////////////////

void FastSerial::begin(long baud)
{
        unsigned int    rxb, txb;

        // If we are re-configuring an already-open port, preserve the
        // existing buffer sizes.
        if (_open) {
                rxb = _rxBuffer->mask + 1;
                txb = _txBuffer->mask + 1;
        } else {
                rxb = RX_BUFFER_SIZE;
                txb = TX_BUFFER_SIZE;
        }

        begin(baud, RX_BUFFER_SIZE, TX_BUFFER_SIZE);
}

void FastSerial::begin(long baud, unsigned int rxSpace, unsigned int txSpace)
{
        uint16_t        ubrr;
        bool            use_u2x = false;
        int             ureg, u2;
        long            breg, b2, dreg, d2;
       
        // if we are currently open, close and restart
        if (_open)
                end();

        // allocate buffers
        if (!_allocBuffer(_rxBuffer, rxSpace ? : RX_BUFFER_SIZE) ||
            !_allocBuffer(_txBuffer, txSpace ? : TX_BUFFER_SIZE)) {
                end();
                return;                 // couldn't allocate buffers - fatal
        }
        _open = true;

        // U2X mode is needed for bitrates higher than (F_CPU / 16)
        if (baud > F_CPU / 16) {
                use_u2x = true;
                ubrr = F_CPU / (8 * baud) - 1;
        } else {
                // Determine whether u2x mode would give a closer
                // approximation of the desired speed.
    
                // ubrr for non-2x mode, corresponding baudrate and delta
                ureg = F_CPU / 16 / baud - 1;
                breg = F_CPU / 16 / (ureg + 1);
                dreg = abs(baud - breg);
                
                // ubrr for 2x mode, corresponding bitrate and delta
                u2   = F_CPU / 8 / baud - 1;
                b2   = F_CPU / 8 / (u2 + 1); 
                d2   = abs(baud - b2);

                // Pick the setting that gives the smallest rate
                // error, preferring non-u2x mode if the delta is
                // identical.
                if (dreg <= d2) {
                        ubrr = ureg;
                } else {
                        ubrr = u2;
                        use_u2x = true;
                }                
        }
  
        *_ucsra = use_u2x ? _BV(_u2x) : 0;
        *_ubrrh = ubrr >> 8;
        *_ubrrl = ubrr;
        *_ucsrb |= _portEnableBits;
}

void FastSerial::end()
{
        *_ucsrb &= ~(_portEnableBits | _portTxBits);

        _freeBuffer(_rxBuffer);
        _freeBuffer(_txBuffer);
        _open = false;
}

int
FastSerial::available(void)
{
        if (!_open)
                return(-1);
        return((_rxBuffer->head - _rxBuffer->tail) & _rxBuffer->mask);
}

int
FastSerial::read(void)
{
        uint8_t         c;

        // if the head and tail are equal, the buffer is empty
        if (!_open || (_rxBuffer->head == _rxBuffer->tail))
                return(-1);

        // pull character from tail
        c = _rxBuffer->bytes[_rxBuffer->tail];
        _rxBuffer->tail = (_rxBuffer->tail + 1) & _rxBuffer->mask;

        return(c);
}

void
FastSerial::flush(void)
{
        // don't reverse this or there may be problems if the RX interrupt
        // occurs after reading the value of _rxBuffer->head but before writing
        // the value to _rxBuffer->tail; the previous value of head
        // may be written to tail, making it appear as if the buffer
        // don't reverse this or there may be problems if the RX interrupt
        // occurs after reading the value of head but before writing
        // the value to tail; the previous value of rx_buffer_head
        // may be written to tail, making it appear as if the buffer
        // were full, not empty.
        _rxBuffer->head = _rxBuffer->tail;

        // don't reverse this or there may be problems if the TX interrupt
        // occurs after reading the value of _txBuffer->tail but before writing
        // the value to _txBuffer->head.
        _txBuffer->tail = _rxBuffer->head;
}

void
FastSerial::write(uint8_t c)
{
        int16_t         i;

        if (!_open)                     // drop bytes if not open
                return;

        // wait for room in the tx buffer
        i = (_txBuffer->head + 1) & _txBuffer->mask;
        while (i == _txBuffer->tail)
                ;

        // add byte to the buffer
        _txBuffer->bytes[_txBuffer->head] = c;
        _txBuffer->head = i;

        // enable the data-ready interrupt, as it may be off if the buffer is empty
        *_ucsrb |= _portTxBits;
}

// STDIO emulation /////////////////////////////////////////////////////////////

int
FastSerial::_putchar(char c, FILE *stream)
{
        FastSerial      *fs;

        fs = (FastSerial *)fdev_get_udata(stream);
        if ('\n' == c)
                fs->write('\r');        // ASCII translation on the cheap
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
        i = vfprintf_P(&_fd, fmt, ap);
        va_end(ap);

        return(i);
}

// Buffer management ///////////////////////////////////////////////////////////

bool
FastSerial::_allocBuffer(Buffer *buffer, unsigned int size)
{
        uint8_t shift;

        // init buffer state
        buffer->head = buffer->tail = 0;

        // Compute the power of 2 greater or equal to the requested buffer size
        // and then a mask to simplify wrapping operations.  Using __builtin_clz
        // would seem to make sense, but it uses a 256(!) byte table.
        // Note that we ignore requests for more than BUFFER_MAX space.
        for (shift = 1; (1U << shift) < min(BUFFER_MAX, size); shift++)
                ;
        buffer->mask = (1 << shift) - 1;

        // allocate memory for the buffer - if this fails, we fail
        buffer->bytes = (uint8_t *)malloc(buffer->mask + 1);

        return(buffer->bytes != NULL);
}

void
FastSerial::_freeBuffer(Buffer *buffer)
{
        buffer->head = buffer->tail = 0;
        buffer->mask = 0;
        if (NULL != buffer->bytes) {
                free(buffer->bytes);
                buffer->bytes = NULL;
        }
}

