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


#include <wiring.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "FastSerial.h"

#if defined(__AVR_ATmega1280__)
# define FS_MAX_PORTS   4
#else
# define FS_MAX_PORTS   1
#endif

static FastSerial       *ports[FS_MAX_PORTS];

#define RX_BUFFER_SIZE  sizeof(((FastSerial::RXBuffer *)1)->bytes)
#define TX_BUFFER_SIZE  sizeof(((FastSerial::TXBuffer *)1)->bytes)

// Interrupt handlers //////////////////////////////////////////////////////////

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

// Constructor /////////////////////////////////////////////////////////////////

FastSerial::FastSerial(uint8_t portNumber)
{
        switch(portNumber) {
#if defined(__AVR_ATmega8__)
        case 0:
                _ubrrh = &UBRRH;
                _ubrrl = &UBRRL;
                _ucsra = &UCSRA;
                _ucsrb = &UCSRB;
                _udr   = &UDR;
                _u2x   = U2X;
                _portEnableBits = _BV(RXEN) |  _BV(TXEN) | _BV(RXCIE);
                _portTxBits     = _BV(UDRIE);
                break;
#else
        case 0:
                _ubrrh = &UBRR0H;
                _ubrrl = &UBRR0L;
                _ucsra = &UCSR0A;
                _ucsrb = &UCSR0B;
                _udr   = &UDR0;
                _u2x   = U2X0;
                _portEnableBits = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
                _portTxBits     = _BV(UDRIE0);
                break;
#if defined(__AVR_ATmega1280__)
        case 1:
                _ubrrh = &UBRR1H;
                _ubrrl = &UBRR1L;
                _ucsra = &UCSR1A;
                _ucsrb = &UCSR1B;
                _udr   = &UDR1;
                _u2x   = U2X1;
                _portEnableBits = _BV(RXEN1) | _BV(TXEN1) | _BV(RXCIE1);
                _portTxBits     = _BV(UDRIE1);
                break;
        case 2:
                _ubrrh = &UBRR2H;
                _ubrrl = &UBRR2L;
                _ucsra = &UCSR2A;
                _ucsrb = &UCSR2B;
                _udr   = &UDR2;
                _u2x   = U2X2;
                _portEnableBits = _BV(RXEN2) | _BV(TXEN2) | _BV(RXCIE2);
                _portTxBits     = _BV(UDRIE2);
                break;
        case 3:
                _ubrrh = &UBRR3H;
                _ubrrl = &UBRR3L;
                _ucsra = &UCSR3A;
                _ucsrb = &UCSR3B;
                _udr   = &UDR3;
                _u2x   = U2X3;
                _portEnableBits = _BV(RXEN3) | _BV(TXEN3) | _BV(RXCIE3);
                _portTxBits     = _BV(UDRIE3);
                break;
#endif
#endif
        default:
                return;
        };

        _txBuffer.head = _txBuffer.tail = 0;
        _rxBuffer.head = _rxBuffer.tail = 0;

        // claim the port
        ports[portNumber] = this;
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

