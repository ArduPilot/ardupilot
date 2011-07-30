/*
	AP_ADC_ADS7844.cpp - ADC ADS7844 Library for Ardupilot Mega
	Code by Jordi Mu�oz and Jose Julio. DIYDrones.com

	Modified by John Ihlein 6 / 19 / 2010 to:
	1)Prevent overflow of adc_counter when more than 8 samples collected between reads.	Probably
		only an issue on initial read of ADC at program start.
	2)Reorder analog read order as follows:
		p, q, r, ax, ay, az

	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

	External ADC ADS7844 is connected via Serial port 2 (in SPI mode)
	TXD2 = MOSI = pin PH1
	RXD2 = MISO = pin PH0
	XCK2 = SCK = pin PH2
	Chip Select pin is PC4 (33)	 [PH6 (9)]
	We are using the 16 clocks per conversion timming to increase efficiency (fast)
	The sampling frequency is 400Hz (Timer2 overflow interrupt)
	So if our loop is at 50Hz, our needed sampling freq should be 100Hz, so
	we have an 4x oversampling and averaging.

	Methods:
		Init() : Initialization of interrupts an Timers (Timer2 overflow interrupt)
		Ch(ch_num) : Return the ADC channel value

	// HJI - Input definitions.  USB connector assumed to be on the left, Rx and servo
	// connector pins to the rear.  IMU shield components facing up.  These are board
	// referenced sensor inputs, not device referenced.
	On Ardupilot Mega Hardware, oriented as described above:
	Chennel 0 : yaw rate, r
	Channel 1 : roll rate, p
	Channel 2 : pitch rate, q
	Channel 3 : x / y gyro temperature
	Channel 4 : x acceleration, aX
	Channel 5 : y acceleration, aY
	Channel 6 : z acceleration, aZ
	Channel 7 : Differential pressure sensor port

*/
extern "C" {
	// AVR LibC Includes
	#include <inttypes.h>
	#include <avr/interrupt.h>
	#include "WConstants.h"
}

#include "AP_ADC_ADS7844.h"


// Commands for reading ADC channels on ADS7844
static const unsigned char 		adc_cmd[9]		= { 0x87, 0xC7, 0x97, 0xD7, 0xA7, 0xE7, 0xB7, 0xF7, 0x00 };
static volatile uint16_t 		_filter[8][ADC_FILTER_SIZE];
static volatile uint8_t			_filter_index;

static unsigned char ADC_SPI_transfer(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR2A & (1 << UDRE2)) );
	/* Put data into buffer, sends the data */
	UDR2 = data;
	/* Wait for data to be received */
	while ( !(UCSR2A & (1 << RXC2)) );
	/* Get and return received data from buffer */
	return UDR2;
}


ISR (TIMER2_OVF_vect)
{
	uint8_t ch;
	uint16_t adc_tmp;

  //bit_set(PORTL,6);                        			  	// To test performance

	bit_clear(PORTC, 4);									// Enable Chip Select (PIN PC4)
	ADC_SPI_transfer(adc_cmd[0]);							// Command to read the first channel

	for (ch = 0; ch < 8; ch++){
		adc_tmp = ADC_SPI_transfer(0) << 8;					// Read first byte
		adc_tmp |= ADC_SPI_transfer(adc_cmd[ch + 1]);		// Read second byte and send next command

		// Fill our Moving average filter
		_filter[ch][_filter_index] = adc_tmp >> 3;
	}

	// increment our filter
	_filter_index++;

	// loop our filter
	if(_filter_index == ADC_FILTER_SIZE)
		_filter_index = 0;


	bit_set(PORTC, 4);										// Disable Chip Select (PIN PC4)
  //bit_clear(PORTL,6);                               		// To test performance
	TCNT2 = 104;											// 400 Hz
}


// Constructors ////////////////////////////////////////////////////////////////
AP_ADC_ADS7844::AP_ADC_ADS7844()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void AP_ADC_ADS7844::Init(void)
{
	pinMode(ADC_CHIP_SELECT, OUTPUT);

	digitalWrite(ADC_CHIP_SELECT, HIGH);								 // Disable device (Chip select is active low)

	// Setup Serial Port2 in SPI mode
	UBRR2 = 0;
	DDRH |= (1 << PH2);																	 // SPI clock XCK2 (PH2) as output. This enable SPI Master mode
	// Set MSPI mode of operation and SPI data mode 0.
	UCSR2C = (1 << UMSEL21) | (1 << UMSEL20);								 // |(0 << UCPHA2) | (0 << UCPOL2);
	// Enable receiver and transmitter.
	UCSR2B = (1 << RXEN2) | (1 << TXEN2);
	// Set Baud rate
	UBRR2 = 2;																					// SPI clock running at 2.6MHz


	// Enable Timer2 Overflow interrupt to capture ADC data
	TIMSK2 = 0;																				 // Disable interrupts
	TCCR2A = 0;																				 // normal counting mode
	TCCR2B = _BV(CS21) | _BV(CS22);											 // Set prescaler of 256
	TCNT2	= 0;
	TIFR2	= _BV(TOV2);																 // clear pending interrupts;
	TIMSK2 = _BV(TOIE2) ;															 // enable the overflow interrupt
}

// Read one channel value
int AP_ADC_ADS7844::Ch(unsigned char ch_num)
{
	uint16_t result = 0;

	//while(adc_counter[ch_num] < 2) { }									// Wait for at least 2 samples in accumlator

	// stop interrupts
	cli();

	// sum our filter
	for(uint8_t i = 0; i < ADC_FILTER_SIZE; i++){
		result += _filter[ch_num][i];
	}

	// resume interrupts
	sei();

	// average our sampels
	result /= ADC_FILTER_SIZE;

	return(result);
}

// Read one channel value
int AP_ADC_ADS7844::Ch_raw(unsigned char ch_num)
{
	return _filter[ch_num][_filter_index]; // close enough
}
