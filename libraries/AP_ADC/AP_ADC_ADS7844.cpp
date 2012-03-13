/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

	The sampling frequency is 1kHz (Timer2 overflow interrupt)

	So if our loop is at 50Hz, our needed sampling freq should be 100Hz, so
	we have an 10x oversampling and averaging.

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
#include "AP_ADC_ADS7844.h"

extern "C" {
	// AVR LibC Includes
	#include <inttypes.h>
	#include <stdint.h>
	#include <avr/interrupt.h>
}
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WConstants.h"
#endif

// Commands for reading ADC channels on ADS7844
static const unsigned char 		adc_cmd[9]		= { 0x87, 0xC7, 0x97, 0xD7, 0xA7, 0xE7, 0xB7, 0xF7, 0x00 };

// the sum of the values since last read
static volatile uint32_t _sum[8];

// how many values we've accumulated since last read
static volatile uint16_t _count[8];

static uint32_t last_ch6_micros;

// TCNT2 values for various interrupt rates,
// assuming 256 prescaler. Note that these values
// assume a zero-time ISR. The actual rate will be a
// bit lower than this
#define TCNT2_781_HZ   (256-80)
#define TCNT2_1008_HZ  (256-62)
#define TCNT2_1302_HZ  (256-48)

static inline unsigned char ADC_SPI_transfer(unsigned char data)
{
	/* Put data into buffer, sends the data */
	UDR2 = data;
	/* Wait for data to be received */
	while ( !(UCSR2A & (1 << RXC2)) );
	/* Get and return received data from buffer */
	return UDR2;
}


void AP_ADC_ADS7844::read(uint32_t tnow)
{
	uint8_t ch;

	bit_clear(PORTC, 4);							// Enable Chip Select (PIN PC4)
	ADC_SPI_transfer(adc_cmd[0]);						// Command to read the first channel

	for (ch = 0; ch < 8; ch++) {
		uint16_t v;

		v = ADC_SPI_transfer(0) << 8;	         // Read first byte
		v |= ADC_SPI_transfer(adc_cmd[ch + 1]);  // Read second byte and send next command

		if (v & 0x8007) {
			// this is a 12-bit ADC, shifted by 3 bits.
			// if we get other bits set then the value is
			// bogus and should be ignored
			continue;
		}

		if (++_count[ch] == 0) {
			// overflow ... shouldn't happen too often
			// unless we're just not using the
			// channel. Notice that we overflow the count
			// to 1 here, not zero, as otherwise the
			// reader below could get a division by zero
			_sum[ch] = 0;
			_count[ch] = 1;
		}
		_sum[ch] += (v >> 3);
	}

	bit_set(PORTC, 4);					// Disable Chip Select (PIN PC4)

}


// Constructors ////////////////////////////////////////////////////////////////
AP_ADC_ADS7844::AP_ADC_ADS7844()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void AP_ADC_ADS7844::Init( AP_PeriodicProcess * scheduler )
{
	pinMode(ADC_CHIP_SELECT, OUTPUT);

	digitalWrite(ADC_CHIP_SELECT, HIGH);  // Disable device (Chip select is active low)

	// Setup Serial Port2 in SPI mode
	UBRR2 = 0;
	DDRH |= (1 << PH2);	// SPI clock XCK2 (PH2) as output. This enable SPI Master mode
	// Set MSPI mode of operation and SPI data mode 0.
	UCSR2C = (1 << UMSEL21) | (1 << UMSEL20); // |(0 << UCPHA2) | (0 << UCPOL2);
	// Enable receiver and transmitter.
	UCSR2B = (1 << RXEN2) | (1 << TXEN2);
	// Set Baud rate
	UBRR2 = 2;	// SPI clock running at 2.6MHz

	// get an initial value for each channel. This ensures
	// _count[] is never zero
	for (uint8_t i=0; i<8; i++) {
		uint16_t adc_tmp;
		adc_tmp  = ADC_SPI_transfer(0) << 8;
		adc_tmp |= ADC_SPI_transfer(adc_cmd[i + 1]);
		_count[i] = 1;
		_sum[i]   = adc_tmp;
	}

	last_ch6_micros = micros();

    scheduler->register_process( AP_ADC_ADS7844::read );

}

// Read one channel value
float AP_ADC_ADS7844::Ch(uint8_t ch_num)
{
	uint16_t count;
	uint32_t sum;

	// ensure we have at least one value
	while (_count[ch_num] == 0)  /* noop */ ;

	// grab the value with interrupts disabled, and clear the count
	cli();
	count = _count[ch_num];
	sum   = _sum[ch_num];
	_count[ch_num] = 0;
	_sum[ch_num]   = 0;
	sei();

	return ((float)sum)/count;
}

// see if Ch6() can return new data
bool AP_ADC_ADS7844::new_data_available(const uint8_t *channel_numbers)
{
	uint8_t i;

	for (i=0; i<6; i++) {
		if (_count[channel_numbers[i]] == 0) {
            return false;
        }
	}
    return true;
}


// Read 6 channel values
// this assumes that the counts for all of the 6 channels are
// equal. This will only be true if we always consistently access a
// sensor by either Ch6() or Ch() and never mix them. If you mix them
// then you will get very strange results
uint32_t AP_ADC_ADS7844::Ch6(const uint8_t *channel_numbers, float *result)
{
	uint16_t count[6];
	uint32_t sum[6];
	uint8_t i;

	// ensure we have at least one value
	for (i=0; i<6; i++) {
		while (_count[channel_numbers[i]] == 0) /* noop */;
	}

	// grab the values with interrupts disabled, and clear the counts
	cli();
	for (i=0; i<6; i++) {
		count[i] = _count[channel_numbers[i]];
		sum[i]   = _sum[channel_numbers[i]];
		_count[channel_numbers[i]] = 0;
		_sum[channel_numbers[i]]   = 0;
	}
	sei();

	// calculate averages. We keep this out of the cli region
	// to prevent us stalling the ISR while doing the
	// division. That costs us 36 bytes of stack, but I think its
	// worth it.
	for (i = 0; i < 6; i++) {
		result[i] = (sum[i] + count[i]) / (float)count[i];
	}

	// return number of microseconds since last call
	uint32_t us = micros();
	uint32_t ret = us - last_ch6_micros;
	last_ch6_micros = us;
	return ret;
}
