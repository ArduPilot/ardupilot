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

#ifndef _FASTUSB_H_
#define _FASTUSB_H_

#include "BetterStream.h"
#include <usb.h>
#include <gpio_hal.h>
#include <wirish.h>

#define DEFAULT_TX_TIMEOUT 10000 // 10 ms

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
//extern class FastSerial Serial;
//@}

/// The FastSerial class definition
///

class FastUSB : public BetterStream {
private:
    usart_dev *usart_device;
    uint8 tx_pin;
    uint8 rx_pin;

public:
    FastUSB();
    FastUSB(usart_dev *usart_device,
               uint8 tx_pin,
               uint8 rx_pin);

    virtual void init(usart_dev *usart_device,
                   uint8 tx_pin,
                   uint8 rx_pin);
    virtual void configure(uint8 port);
//	virtual void begin(uint32  baudRate,
//					   uint16  wordLength,
//					   uint16  stopBits,
//					   uint16  parity,
//					   uint16  mode,
//					   uint16  hardwareFlowControl);
    virtual void begin(long baud);
    virtual void begin(long baud, uint32_t tx_timeout);
    virtual void end(void);
    virtual int available(void);
	virtual int txspace(void);
    virtual int read(void);
	virtual int peek(void);
    virtual void flush(void);
    virtual void write(uint8_t c);
    virtual void use_tx_fifo(bool enable);
    virtual void use_timeout(uint8_t enable);
    virtual void set_timeout(uint32_t timeout);
    virtual uint32_t txfifo_nbytes(void);
    virtual uint32_t txfifo_freebytes(void);
    virtual void set_blocking_writes(bool enable);
	
	//kept for compatibility:
	virtual void begin(long baud, unsigned int rxSpace, unsigned int txSpace);
	virtual long getPortLic(void);
    using BetterStream::write;
};

//TEO 20110505
//definisco qui i parametri per le varie seriali preconfigurate
#define FSUSART0 	_USART1
#define FSTXPIN0 	BOARD_USART1_TX_PIN //9
#define FSRXPIN0 	BOARD_USART1_RX_PIN //10

#define FSUSART1 	_USART2
#define FSTXPIN1 	BOARD_USART2_TX_PIN //5
#define FSRXPIN1 	BOARD_USART2_RX_PIN //6

#define FSUSART2 	_USART3
#define FSTXPIN2 	BOARD_USART3_TX_PIN //8
#define FSRXPIN2 	BOARD_USART3_RX_PIN //9

#define FSUSART3 	_UART4
#define FSTXPIN3 	BOARD_UART4_TX_PIN //10
#define FSRXPIN3 	BOARD_UART4_RX_PIN //11

#define FastUSBPort(_name)    FastUSB _name();
/*
///
/// Macro defining a FastSerial port instance.
///
#define FastSerialPort(_name, _num)   		\
		FastSerial _name(FSUSART##_num, 	\
						FSTXPIN##_num, 		\
						FSRXPIN##_num);

#define FastSerialPort_Init(_name, _num)   	\
		_name.init(FSUSART##_num,		\
						 FSTXPIN##_num, 	\
						 FSRXPIN##_num);


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
*/
/*
#define FastSerialPort_Init0(_portName)     FastSerialPort_Init(_portName, 0)
#define FastSerialPort_Init1(_portName)     FastSerialPort_Init(_portName, 1)
#define FastSerialPort_Init2(_portName)     FastSerialPort_Init(_portName, 2)
#define FastSerialPort_Init3(_portName)     FastSerialPort_Init(_portName, 3)
*/

// TODO: high density device ports
#endif

