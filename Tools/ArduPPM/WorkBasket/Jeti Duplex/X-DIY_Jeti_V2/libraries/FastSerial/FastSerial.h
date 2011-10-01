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
//      version 2.1 of the License, or (at your option) any later
//      version.
//
//      This library is distributed in the hope that it will be
//      useful, but WITHOUT ANY WARRANTY; without even the implied
//      warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//      PURPOSE.  See the GNU Lesser General Public License for more
//      details.
//
//      You should have received a copy of the GNU Lesser General
//      Public License along with this library; if not, write to the
//      Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
//      Boston, MA 02110-1301 USA
//

//
// Note that this library does not pre-declare drivers for serial
// ports; the user must explicitly create drivers for the ports they
// wish to use.  This is less friendly than the stock Arduino driver,
// but it saves ~200 bytes for every unused port.
//

#ifndef FastSerial_h
#define FastSerial_h

// disable the stock Arduino serial driver
#ifdef HardwareSerial_h
# error Must include FastSerial.h before the Arduino serial driver is defined.
#endif
#define HardwareSerial_h

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <Stream.h>
#include <avr/interrupt.h>

//
// Because Arduino libraries aren't really libraries, but we want to
// only define interrupt handlers for serial ports that are actually
// used, we have to force our users to define them using a macro.
//
// Due to the way interrupt vectors are specified, we have to have
// a separate macro for every port.  Ugh.
//
// The macros are:
//
// FastSerialPort0(<port name>)         creates <port name> referencing serial port 0
// FastSerialPort1(<port name>)         creates <port name> referencing serial port 1
// FastSerialPort2(<port name>)         creates <port name> referencing serial port 2
// FastSerialPort3(<port name>)         creates <port name> referencing serial port 3
//

//
// Forward declarations for clients that want to assume that the
// default Serial* objects exist.
//
// Note that the application is responsible for ensuring that these
// actually get defined, otherwise Arduino will suck in the
// HardwareSerial library and linking will fail.
//
extern class FastSerial Serial;
extern class FastSerial Serial1;
extern class FastSerial Serial2;
//extern class FastSerial Serial3;


class FastSerial : public Stream {
public:
        FastSerial(const uint8_t portNumber,
                   volatile uint8_t *ubrrh,
                   volatile uint8_t *ubrrl,
                   volatile uint8_t *ucsra,
                   volatile uint8_t *ucsrb,
                   volatile uint8_t *udr,
                   const uint8_t u2x,
                   const uint8_t portEnableBits,
                   const uint8_t portTxBits);

        // Serial API
        void            begin(long baud);
        void            begin(long baud, unsigned int rxSpace, unsigned int txSpace);
        void            end(void);
        int             available(void);
        int             read(void);
        void            flush(void);
        void            write(uint8_t c);
        using Stream::write;

        // stdio extensions
        int             printf(const char *fmt, ...);
        int             printf_P(const char *fmt, ...);
        FILE            *getfd(void) { return &_fd; };

        // public so the interrupt handlers can see it
        struct Buffer {
                volatile uint16_t head, tail;
                uint16_t        mask;
                uint8_t         *bytes;
        };

private:
        // register accessors
        volatile uint8_t *_ubrrh;
        volatile uint8_t *_ubrrl;
        volatile uint8_t *_ucsra;
        volatile uint8_t *_ucsrb;
        volatile uint8_t *_udr;

        // register magic numbers
        uint8_t         _portEnableBits;        // rx, tx and rx interrupt enables
        uint8_t         _portTxBits;            // tx data and completion interrupt enables
        uint8_t         _u2x;

        // ring buffers
        Buffer          *_rxBuffer;
        Buffer          *_txBuffer;
        bool            _open;

        bool            _allocBuffer(Buffer *buffer, unsigned int size);
        void            _freeBuffer(Buffer *buffer);

        // stdio emulation
        FILE            _fd;
        static int      _putchar(char c, FILE *stream);
        static int      _getchar(FILE *stream);
};

// Used by the per-port interrupt vectors
extern FastSerial::Buffer	__FastSerial__rxBuffer[];
extern FastSerial::Buffer	__FastSerial__txBuffer[];

// Generic Rx/Tx vectors for a serial port - needs to know magic numbers
#define FastSerialHandler(_PORT, _RXVECTOR, _TXVECTOR, _UDR, _UCSRB, _TXBITS) \
ISR(_RXVECTOR, ISR_BLOCK)                                               \
{                                                                       \
        uint8_t c;                                                      \
        int16_t i;                                                      \
                                                                        \
        /* read the byte as quickly as possible */                      \
        c = _UDR;                                                       \
        /* work out where the head will go next */                      \
        i = (__FastSerial__rxBuffer[_PORT].head + 1) & __FastSerial__rxBuffer[_PORT].mask; \
        /* decide whether we have space for another byte */             \
        if (i != __FastSerial__rxBuffer[_PORT].tail) {                  \
                /* we do, move the head */                              \
                __FastSerial__rxBuffer[_PORT].bytes[__FastSerial__rxBuffer[_PORT].head] = c; \
                __FastSerial__rxBuffer[_PORT].head = i;                 \
        }                                                               \
}                                                                       \
ISR(_TXVECTOR, ISR_BLOCK)                                               \
{                                                                       \
        /* if there is another character to send */                     \
        if (__FastSerial__txBuffer[_PORT].tail != __FastSerial__txBuffer[_PORT].head) { \
                _UDR = __FastSerial__txBuffer[_PORT].bytes[__FastSerial__txBuffer[_PORT].tail]; \
                /* increment the tail */                                \
                __FastSerial__txBuffer[_PORT].tail =                    \
                        (__FastSerial__txBuffer[_PORT].tail + 1) & __FastSerial__txBuffer[_PORT].mask; \
        } else {                                                        \
                /* there are no more bytes to send, disable the interrupt */ \
                if (__FastSerial__txBuffer[_PORT].head == __FastSerial__txBuffer[_PORT].tail) \
                        _UCSRB &= ~_TXBITS;                             \
        }                                                               \
}                                                                       \
struct hack

// Macros defining serial ports
#if defined(__AVR_ATmega1280__)
#define FastSerialPort0(_portName)                                      \
	FastSerial _portName(0,                                       		\
                             &UBRR0H,                                   \
                             &UBRR0L,                                   \
                             &UCSR0A,                                   \
                             &UCSR0B,                                   \
                             &UDR0,                                     \
                             U2X0,                                      \
                             (_BV(RXEN0) |  _BV(TXEN0) | _BV(RXCIE0)),  \
                             (_BV(UDRIE0)));                            \
	FastSerialHandler(0, SIG_USART0_RECV, SIG_USART0_DATA, UDR0, UCSR0B, _BV(UDRIE0))
#define FastSerialPort1(_portName)                                      \
	FastSerial _portName(1,                                         	\
                             &UBRR1H,                                   \
                             &UBRR1L,                                   \
                             &UCSR1A,                                   \
                             &UCSR1B,                                   \
                             &UDR1,                                     \
                             U2X1,                                      \
                             (_BV(RXEN1) |  _BV(TXEN1) | _BV(RXCIE1)),  \
                             (_BV(UDRIE1)));                            \
	FastSerialHandler(1, SIG_USART1_RECV, SIG_USART1_DATA, UDR1, UCSR1B, _BV(UDRIE1))
#define FastSerialPort2(_portName)                                      \
	FastSerial _portName(2,                                         	\
                             &UBRR2H,                                   \
                             &UBRR2L,                                   \
                             &UCSR2A,                                   \
                             &UCSR2B,                                   \
                             &UDR2,                                     \
                             U2X2,                                      \
                             (_BV(RXEN2) |  _BV(TXEN2) | _BV(RXCIE2)),  \
                             (_BV(UDRIE2)));                            \
	FastSerialHandler(2, SIG_USART2_RECV, SIG_USART2_DATA, UDR2, UCSR2B, _BV(UDRIE2))
/*
	#define FastSerialPort3(_portName)                                      \
	FastSerial _portName(3,                                         	\
                             &UBRR3H,                                   \
                             &UBRR3L,                                   \
                             &UCSR3A,                                   \
                             &UCSR3B,                                   \
                             &UDR3,                                     \
                             U2X3,                                      \
                             (_BV(RXEN3) |  _BV(TXEN3) | _BV(RXCIE3)),  \
                             (_BV(UDRIE3)));                            \
	FastSerialHandler(3, SIG_USART3_RECV, SIG_USART3_DATA, UDR3, UCSR3B, _BV(UDRIE3))
*/
	#else
#if defined(__AVR_ATmega8__)
#define FastSerialPort0(_portName)                                      \
	FastSerial _portName(0,                                        		\
                             &UBRR0H,                                   \
                             &UBRR0L,                                   \
                             &UCSR0A,                                   \
                             &UCSR0B,                                   \
                             &UDR0,                                     \
                             U2X0,                                      \
                             (_BV(RXEN0) |  _BV(TXEN0) | _BV(RXCIE0)),  \
                             (_BV(UDRIE0)));                            \
	FastSerialHandler(0, SIG_UART_RECV, SIG_UART_DATA, UDR0, UCSR0B, _BV(UDRIE0))
#else
// note no SIG_USART_* defines for the 168, 328, etc.
#define FastSerialPort0(_portName)                                      \
	FastSerial _portName(0,                                         	\
                             &UBRR0H,                                   \
                             &UBRR0L,                                   \
                             &UCSR0A,                                   \
                             &UCSR0B,                                   \
                             &UDR0,                                     \
                             U2X0,                                      \
                             (_BV(RXEN0) |  _BV(TXEN0) | _BV(RXCIE0)),  \
                             (_BV(UDRIE0)));                            \
	FastSerialHandler(0, USART_RX_vect, USART_UDRE_vect, UDR0, UCSR0B, _BV(UDRIE0))
#endif
#endif
#endif // FastSerial_h
