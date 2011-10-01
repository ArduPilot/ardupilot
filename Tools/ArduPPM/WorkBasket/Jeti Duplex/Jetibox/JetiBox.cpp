/*
  JetiBox.cpp, Version 1.0 beta
  July 2010, by Uwe Gartmann

  Library acts as a Sensor when connected to a Jeti Receiver
  written for Arduino Mega / ArduPilot Mega

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "wiring.h"
#include "mySerial0.h"
#include "JetiBox.h"

#define LCDLine1 1
#define LCDLine2 17
#define LCDMaxChar 32

//define Jeti-Box Display
struct jeti_box {
  unsigned char lcd[32];
  volatile unsigned char lcdpos;
  volatile unsigned char keys;
  volatile unsigned char sendpos;
};

// create jbox
jeti_box jb = {{0},0,0,0};
unsigned char dummy;
unsigned char lastkey = 0;

ISR(USART3_RX_vect)		//serial data available
{
	// save response data to keys
	dummy = UDR3;
	if (dummy != 0xFF)
	{
		jb.keys = dummy;
		// disable this interrupt
		UCSR3B &= ~(1<<RXEN3);
	}
}

ISR(USART3_UDRE_vect)	//transmit buffer empty
/*
 * jbox.sendpos = 0 -> interrupt is enabled (function start() ), send startbyte with 9.bit=0
 * jbox.sendpos = 1-32 -> send display data with 9.bit=1
 * jbox.sendpos = 33 -> send endbyte with 9.bit=0
 * jbox.sendpos = 34 -> reset sendpos=0 -> disable this interrupt
 */
{
	switch (jb.sendpos)
	{
		case 0:
			// send start byte with 9.bit=0
			UCSR3B &= ~(1<<TXB83);  // clear bit 9
			UDR3 = 0xFE;
			jb.sendpos++;
			break;

		case 33:
			// send end byte with 9.bit=0
			UCSR3B &= ~(1<<TXB83);
			UDR3 = 0xFF;
			jb.sendpos++;
			break;

		case 34:
			// disable interrupt transmit buffer empty
			UCSR3B &= ~(1<<UDRIE3);
			// set sendpos -> 0
			jb.sendpos = 0;
			// enable receiver interrupt for reading key byte
			UCSR3B |= (1<<RXEN3);
			break;

		default:
			// set 9.bit=1
			UCSR3B |= (1<<TXB83);
			// send byte from LCD buffer
			UDR3 = jb.lcd[jb.sendpos];
			// increment to next byte
			jb.sendpos++;
	}
}

JetiBox::JetiBox()
{
// Class constructor
}

// Public Methods --------------------------------------------------------------------
void JetiBox::begin()
{
#ifndef F_CPU
	#define F_CPU 16000000
#endif
#define _UBRR3 (F_CPU/8/9600-1)	//when U2X0 is not set use divider 16 instead of 8

	// Set baudrate
	UCSR3A = (1<<U2X3); //U2X0 enabled!
	UBRR3H=(_UBRR3>>8);   //high byte
	UBRR3L=_UBRR3;      //low byte

	// Set frame format: 9data, odd parity, 2stop bit
	UCSR3C = (0<<UMSEL30)|(3<<UPM30)|(1<<USBS3)|(3<<UCSZ30);

	// Enable receiver and transmitter, set frame size
	UCSR3B = (1<<TXEN3)|(1<<RXCIE3)|(1<<UCSZ32);
}

void JetiBox::refresh()
{
	UCSR3B |= (1<<UDRIE3); // Enable Interrupt
}

uint8_t JetiBox::keys(void)
{
	unsigned char c = (jb.keys>>4) xor 0x0F;
	if (lastkey==c)
	{
		return 0;
	}else{
		return lastkey = c;
	}

}

void JetiBox::write(uint8_t c)
{
	jb.lcd[jb.lcdpos] = c;
	if (jb.lcdpos < LCDMaxChar)
	{
		jb.lcdpos++;
	}
}

void JetiBox::line1()
{
	jb.lcdpos = LCDLine1;
}

void JetiBox::line2()
{
	jb.lcdpos = LCDLine2;
}

// Preinstantiate Objects //////////////////////////////////////////////////////
JetiBox JBox;



