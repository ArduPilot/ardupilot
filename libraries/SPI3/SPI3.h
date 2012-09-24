/*
 *       SPI3.cpp - SPI library using UART3 for Ardupilot Mega
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 */

#ifndef _SPI3_H_INCLUDED
#define _SPI3_H_INCLUDED

#include <FastSerial.h>
#include <avr/pgmspace.h>

// SPI3's standard pins on Atmega2560
#define SPI3_SCK        PJ2
#define SPI3_MOSI       14
#define SPI3_MISO       15

// used to change UART into SPI mode
#define SPI3_USART_MASK         0xC0      // UMSEL31 | UMSEL30
#define SPI3_USART_ASYNC_UART   0x00      // UMSEL31 = 0, UMSEL30 = 0
#define SPI3_USART_SYNC_UART    0x40      // UMSEL31 = 0, UMSEL30 = 1
#define SPI3_USART_SPI_MASTER   0xC0      // UMSEL31 = 1, UMSEL30 = 1

// spi bus speeds.  these assume a 16Mhz clock
#define SPI3_SPEED_8MHZ 0x00
#define SPI3_SPEED_2MHZ 0x04

// default speed
#define SPI3_DEFAULT_SPEED SPI3_SPEED_2MHZ // 2 megahertz

// SPI mode definitions
#define SPI3_MODE_MASK 0x03
#define SPI3_MODE0 0x00
#define SPI3_MODE1 0x01
#define SPI3_MODE2 0x02
#define SPI3_MODE3 0x03
#define SPI3_DEFAULT_MODE SPI3_MODE0

#define SPI3_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI3_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR

// SPI bit order
#define SPI3_MSBFIRST 0x00
#define SPI3_LSBFIRST 0x01

class SPI3Class {
public:

    static void begin();
    static void end();
    static uint8_t transfer(byte data);

    // SPI Configuration methods
    static void setBitOrder(uint8_t bitOrder);
    static void setDataMode(uint8_t mode);
    static void setSpeed(uint8_t rate);

private:
    static bool _initialised;
};

extern SPI3Class SPI3;

#endif
