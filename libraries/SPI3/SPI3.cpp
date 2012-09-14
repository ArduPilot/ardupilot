/*
 *       SPI3.cpp - SPI library using UART3 for Ardupilot Mega
 *       Code by Randy Mackay, DIYDrones.com 
 *       but mostly based on standard Arduino SPI class by Cristian Maglie <c.maglie@bug.st>
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 */

#include "pins_arduino.h"
#include "SPI3.h"

SPI3Class SPI3;
bool SPI3Class::_initialised = false;

void SPI3Class::begin() {

    // check if begin has been run already
    if( _initialised ) {
        return;
    }

    // Set direction register for SCK and MOSI pin.
    pinMode(SPI3_SCK, OUTPUT);
    pinMode(SPI3_MOSI, OUTPUT);
    pinMode(SPI3_MISO, INPUT);

    // Setup Serial Port3 in SPI mode (MSPI), Mode 0, Clock: 8Mhz
    UBRR3 = 0;
    DDRJ |= (1<<PJ2);   // SPI clock XCK3 (PJ2) as output. This enable SPI Master mode

    // put UART3 into SPI master mode, default mode and LSB bit order 
    UCSR3C = SPI3_USART_SPI_MASTER | SPI3_DEFAULT_MODE;

    // Enable receiver and transmitter.
    UCSR3B = (1<<RXEN3)|(1<<TXEN3);

    // Set Baud rate
    UBRR3 = SPI3_DEFAULT_SPEED;    // SPI running at 2Mhz by default

    // initialisation complete
    _initialised = true;
}

// end - switch UART3 back to asyncronous UART
// Note: this is untested
void SPI3Class::end() {
    uint8_t temp = UCSR3C;
    
    // put UART3 into ASync UART mode
    temp = (temp & ~SPI3_USART_MASK) | SPI3_USART_ASYNC_UART;
    UCSR3C = temp;

    // reinitialisation will be required
    _initialised = false;
}

uint8_t SPI3Class::transfer(uint8_t data) {
    /* Wait for empty transmit buffer */
    while ( !( UCSR3A & (1<<UDRE3)) ) ;

    /* Put data into buffer, sends the data */
    UDR3 = data;

    /* Wait for data to be received */
    while ( !(UCSR3A & (1<<RXC3)) ) ;

    /* Get and return received data from buffer */
    return UDR3;
}

void SPI3Class::setBitOrder(uint8_t bitOrder)
{
  if(bitOrder == SPI3_LSBFIRST) {
    UCSR3C |= _BV(2);
  } else {
    UCSR3C &= ~(_BV(2));
  }
}

void SPI3Class::setDataMode(uint8_t mode)
{
  UCSR3C = (UCSR3C & ~SPI3_MODE_MASK) | mode;
}

void SPI3Class::setSpeed(uint8_t rate)
{
  UBRR3 = rate;
}

