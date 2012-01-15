/*
  JETI_Box.cpp, Version 1.0 beta
  Oct 2010, by Uwe Gartmann


  written for Arduino Mega / ArduPilot Mega

*/
extern "C" {
	#include <inttypes.h>
	#include "Print.h"
	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include "wiring.h"
}
	
#include <JETI_Box.h>
#define LCDMaxPos 32
#define LCDBufferSize LCDMaxPos + 1

volatile uint8_t lcd[LCDBufferSize];
volatile uint8_t buttons;
volatile uint8_t lastbuttons;
volatile uint8_t cursor;
volatile uint8_t sendpos;
volatile uint8_t dmyError;
volatile uint8_t dmyBit9;
volatile uint8_t dmyData;
volatile uint8_t ledA;
volatile uint8_t t0cntr;
//volatile long valX;

ISR(USART3_RX_vect)		//serial data available
{
	// catch response from Jetibox
	dmyError = UCSR3A;
	dmyBit9 = UCSR3B;
	dmyData = UDR3;
	if (!(dmyError & ((1<<FE3) | (1<<DOR3) | (1<<UPE3)))) // check for frame error, data overflow, parity error
	{
		if ((dmyBit9 & (1<<RXB83)) == 0) //  check bit 9=0 for control data
		{
			if ((dmyData & 0x0F) == 0) // check for plausible button data: mask the 0x?0
			{
				buttons = (dmyData>>4) xor 0x0F;
				t0cntr = 10; // set 10ms delay
			} 
		}
	}
}

ISR(USART3_UDRE_vect)	//transmit buffer empty
/*
 * sendpos = 0 -> start sending data with this interrupt enabled, start byte with 9.bit=0
 * sendpos = 1-32 -> send display data with 9.bit=1
 * sendpos = 33 -> send end byte with 9.bit=0
 * sendpos = 34 -> set sendpos=0 and disable this interrupt
 */
{
	switch (sendpos)
	{
		case 0:
			// send start byte with 9.bit=0
			UCSR3B &= ~(1<<TXB83);  // clear bit 9
			UDR3 = 0xFE;
			sendpos++;
			break;

		case 33:
			// send end byte with 9.bit=0
			UCSR3B &= ~(1<<TXB83);
			UDR3 = 0xFF;
			sendpos++;
			break;

		case 34:
			// all data send, disable send buffer empty interrupt 
			UCSR3B &= ~(1<<UDRIE3);
			// reset sendpos to 0
			sendpos = 0;
			break;

		default:
			// set 9.bit=1
			UCSR3B |= (1<<TXB83);
			// send byte from LCD buffer
			UDR3 = lcd[sendpos];
			// increment to next byte
			sendpos++;
	}
}

ISR(TIMER0_COMPA_vect) // Timer 0 CompA event
// This timer interrupt delays sending the next frame to the Jeti Box (in ms)
// The main function of timer0 belongs to millis() 
{
	switch (t0cntr) {
		case 0: 
			break;
		case 1: 
			--t0cntr;
			UCSR3B |= (1<<UDRIE3); // enable send_buffer_empty interrupt to start transmission
			break;
		default: 
			--t0cntr;
	}
	//if (valX > 0) --valX;
}

JETI_Box_class::JETI_Box_class()
{
// Class constructor
}

// Public Methods --------------------------------------------------------------------
void JETI_Box_class::begin(void)
{
#ifndef F_CPU
	#define F_CPU 16000000
#endif
#define _UBRR3 (F_CPU/8/9600-1)	//when U2X0 is not set use divider 16 instead of 8

	// Set serial interface
	// Baudrate
	UCSR3A = (1<<U2X3); //U2X0 enabled!
	UBRR3H=(_UBRR3>>8); //high byte
	UBRR3L=_UBRR3;      //low byte
	// Set data format: 9data#1, odd parity, 1stop bit
	UCSR3C = (0<<UMSEL30)|(3<<UPM30)|(0<<USBS3)|(3<<UCSZ30);
	// Enable receiver and transmitter, receiver interrupt, set 9data#2 
	UCSR3B = (1<<TXEN3)|(1<<RXEN3)|(1<<RXCIE3)|(1<<UCSZ32);
	
	// config additional timer 0 interrupts (hack)
	OCR0A = 128;			// set interrupt between overflow interrupts
	TIMSK0 |= _BV(OCIE0A);	// enable OCRA overflow interrupt

	clear(0);				// clear lcd[buffer]
}

//long JETI_Box_class::checkvalue(long v) {
//	if (v != 0) valX = v;
//	return valX;
//}

uint8_t JETI_Box_class::readbuttons(void) {
	if (lastbuttons != buttons) {
		return lastbuttons = buttons;
	} else {
		return 0;
	}
}

void JETI_Box_class::write(uint8_t c) {
	lcd[cursor] = c;
	if (cursor < LCDMaxPos)
	{
		cursor++;
	}
}

void JETI_Box_class::setcursor(uint8_t p) {
	cursor = p;
}

void JETI_Box_class::clear() {
	clear(0);
}

void JETI_Box_class::clear(uint8_t l) {
	uint8_t p;
	switch (l)
	{
	case 1: // line1
		for(p = LCDLine1; p < LCDLine2; p++)
		{
			lcd[p] = 32;
		}
		cursor = LCDLine1;
		break;
		
	case 2: // line 2
		for(p = LCDLine2; p <= LCDMaxPos; p++)
		{
			lcd[p] = 32;
		}
		cursor = LCDLine2;
		break;
		
	default: // complete lcd
		for(p = LCDLine1; p <= LCDMaxPos; p++)
		{
			lcd[p] = 32;
		}		
		cursor = LCDLine1;
	}
}

// Preinstantiate Objects //////////////////////////////////////////////////////
JETI_Box_class JB;



