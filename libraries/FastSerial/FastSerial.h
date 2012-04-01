// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
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
// but it saves a few bytes of RAM for every unused port and frees up
// the vector for another driver (e.g. MSPIM on USARTs).
//

#ifndef FastSerial_h
#define FastSerial_h

// disable the stock Arduino serial driver
#ifdef HardwareSerial_h
# error Must include FastSerial.h before the Arduino serial driver is defined.
#endif
#define HardwareSerial_h

#include <inttypes.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "BetterStream.h"


/// @file	FastSerial.h
/// @brief	An enhanced version of the Arduino HardwareSerial class
///			implementing interrupt-driven transmission and flexible
///			buffer management.
///
/// Because Arduino libraries aren't really libraries, but we want to
/// only define interrupt handlers for serial ports that are actually
/// used, we have to force our users to define them using a macro.
///
/// FastSerialPort(<port name>, <port number>)
///
/// <port name> is the name of the object that will be created by the
/// macro.  <port number> is the 0-based number of the port that will
/// be managed by the object.
///
/// Previously ports were defined with a different macro for each port,
/// and these macros are retained for compatibility:
///
/// FastSerialPort0(<port name>)         creates <port name> referencing serial port 0
/// FastSerialPort1(<port name>)         creates <port name> referencing serial port 1
/// FastSerialPort2(<port name>)         creates <port name> referencing serial port 2
/// FastSerialPort3(<port name>)         creates <port name> referencing serial port 3
///
/// Note that compatibility macros are only defined for ports that
/// exist on the target device.
///

///	@name	Compatibility
///
/// Forward declarations for clients that want to assume that the
/// default Serial* objects exist.
///
/// Note that the application is responsible for ensuring that these
/// actually get defined, otherwise Arduino will suck in the
/// HardwareSerial library and linking will fail.
//@{
extern class FastSerial Serial;
extern class FastSerial Serial1;
extern class FastSerial Serial2;
extern class FastSerial Serial3;
//@}

/// The FastSerial class definition
///
class FastSerial: public BetterStream {
public:

	/// Constructor
	FastSerial(const uint8_t portNumber, volatile uint8_t *ubrrh, volatile uint8_t *ubrrl, volatile uint8_t *ucsra,
			   volatile uint8_t *ucsrb, const uint8_t u2x, const uint8_t portEnableBits, const uint8_t portTxBits);

	/// @name 	Serial API
	//@{
	virtual void begin(long baud);
	virtual void end(void);
	virtual int available(void);
	virtual int txspace(void);
	virtual int read(void);
	virtual int peek(void);
	virtual void flush(void);
#if defined(ARDUINO) && ARDUINO >= 100
	virtual size_t write(uint8_t c);
#else
	virtual void write(uint8_t c);
#endif
	using BetterStream::write;
	//@}

	/// Extended port open method
	///
	/// Allows for both opening with specified buffer sizes, and re-opening
	/// to adjust a subset of the port's settings.
	///
	/// @note	Buffer sizes greater than ::_max_buffer_size will be rounded
	///			down.
	///
	/// @param	baud		Selects the speed that the port will be
	///						configured to.  If zero, the port speed is left
	///						unchanged.
	/// @param rxSpace		Sets the receive buffer size for the port.  If zero
	///						then the buffer size is left unchanged if the port
	///						is open, or set to ::_default_rx_buffer_size if it is
	///						currently closed.
	/// @param txSpace		Sets the transmit buffer size for the port.  If zero
	///						then the buffer size is left unchanged if the port
	///						is open, or set to ::_default_tx_buffer_size if it
	///						is currently closed.
	///
	virtual void begin(long baud, unsigned int rxSpace, unsigned int txSpace);

	/// Transmit/receive buffer descriptor.
	///
	/// Public so the interrupt handlers can see it
	struct Buffer {
		volatile uint16_t head, tail;	///< head and tail pointers
		uint16_t mask;					///< buffer size mask for pointer wrap
		uint8_t *bytes;					///< pointer to allocated buffer
	};

	/// Tell if the serial port has been initialized
	static bool getInitialized(uint8_t port) {
		return (1<<port) & _serialInitialized;
	}

	// ask for writes to be blocking or non-blocking
	void set_blocking_writes(bool blocking) {
		_nonblocking_writes = !blocking;
	}

private:

	/// Bit mask for initialized ports
	static uint8_t _serialInitialized;

	/// Set if the serial port has been initialized
	static void setInitialized(uint8_t port) {
		_serialInitialized |= (1<<port);
	}

	// register accessors
	volatile uint8_t * const _ubrrh;
	volatile uint8_t * const _ubrrl;
	volatile uint8_t * const _ucsra;
	volatile uint8_t * const _ucsrb;

	// register magic numbers
	const uint8_t	_u2x;
	const uint8_t	_portEnableBits;		///< rx, tx and rx interrupt enables
	const uint8_t	_portTxBits;			///< tx data and completion interrupt enables


	// ring buffers
	Buffer			* const _rxBuffer;
	Buffer			* const _txBuffer;
	bool 			_open;

	// whether writes to the port should block waiting
	// for enough space to appear
	bool			_nonblocking_writes;

	/// Allocates a buffer of the given size
	///
	/// @param	buffer		The buffer descriptor for which the buffer will
	///						will be allocated.
	/// @param	size		The desired buffer size.
	/// @returns			True if the buffer was allocated successfully.
	///
	static bool _allocBuffer(Buffer *buffer, unsigned int size);

	/// Frees the allocated buffer in a descriptor
	///
	/// @param	buffer		The descriptor whose buffer should be freed.
	///
	static void _freeBuffer(Buffer *buffer);

	/// default receive buffer size
	static const unsigned int	_default_rx_buffer_size = 128;

	/// default transmit buffer size
	static const unsigned int	_default_tx_buffer_size = 16;

	/// maxium tx/rx buffer size
	/// @note if we could bring the max size down to 256, the mask and head/tail
	///       pointers in the buffer could become uint8_t.
	///
	static const unsigned int	_max_buffer_size = 512;
};

// Used by the per-port interrupt vectors
extern FastSerial::Buffer __FastSerial__rxBuffer[];
extern FastSerial::Buffer __FastSerial__txBuffer[];

/// Generic Rx/Tx vectors for a serial port - needs to know magic numbers
///
#define FastSerialHandler(_PORT, _RXVECTOR, _TXVECTOR, _UDR, _UCSRB, _TXBITS) \
ISR(_RXVECTOR, ISR_BLOCK)                                               \
{                                                                       \
        uint8_t c;                                                      \
        uint16_t i;                                                      \
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

//
// Portability; convert various older sets of defines for U(S)ART0 up
// to match the definitions for the 1280 and later devices.
//
#if !defined(USART0_RX_vect)
# if defined(USART_RX_vect)
#  define USART0_RX_vect        USART_RX_vect
#  define USART0_UDRE_vect      USART_UDRE_vect
# elif defined(UART0_RX_vect)
#  define USART0_RX_vect        UART0_RX_vect
#  define USART0_UDRE_vect      UART0_UDRE_vect
# endif
#endif

#if !defined(USART1_RX_vect)
# if defined(UART1_RX_vect)
#  define USART1_RX_vect        UART1_RX_vect
#  define USART1_UDRE_vect      UART1_UDRE_vect
# endif
#endif

#if !defined(UDR0)
# if defined(UDR)
#  define UDR0                  UDR
#  define UBRR0H                UBRRH
#  define UBRR0L                UBRRL
#  define UCSR0A                UCSRA
#  define UCSR0B                UCSRB
#  define U2X0                  U2X
#  define RXEN0                 RXEN
#  define TXEN0                 TXEN
#  define RXCIE0                RXCIE
#  define UDRIE0                UDRIE
# endif
#endif

///
/// Macro defining a FastSerial port instance.
///
#define FastSerialPort(_name, _num)                                     \
	FastSerial _name(_num,                                          \
                         &UBRR##_num##H,                                \
                         &UBRR##_num##L,                                \
                         &UCSR##_num##A,                                \
                         &UCSR##_num##B,                                \
                         U2X##_num,                                     \
                         (_BV(RXEN##_num) |  _BV(TXEN##_num) | _BV(RXCIE##_num)), \
                         (_BV(UDRIE##_num)));                           \
	FastSerialHandler(_num,                                         \
                          USART##_num##_RX_vect,                        \
                          USART##_num##_UDRE_vect,                      \
                          UDR##_num,                                    \
                          UCSR##_num##B,                                \
                          _BV(UDRIE##_num))

///
/// Compatibility macros for previous FastSerial versions.
///
/// Note that these are not conditionally defined, as the errors
/// generated when using these macros for a board that does not support
/// the port are better than the errors generated for a macro that's not
/// defined at all.
///
#define FastSerialPort0(_portName)     FastSerialPort(_portName, 0)
#define FastSerialPort1(_portName)     FastSerialPort(_portName, 1)
#define FastSerialPort2(_portName)     FastSerialPort(_portName, 2)
#define FastSerialPort3(_portName)     FastSerialPort(_portName, 3)

#endif // FastSerial_h
