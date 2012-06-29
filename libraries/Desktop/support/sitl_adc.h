/*
  ADS7844 register emulation
  Code by Andrew Tridgell November 2011
 */

#ifndef _SITL_ADC_H
#define _SITL_ADC_H

#include <stdlib.h>
#include <math.h>
#include "util.h"

// this implements the UDR2 register
struct ADC_UDR2 {
	uint16_t value, next_value;
	uint8_t idx;
	float channels[8];

	ADC_UDR2() {
		// constructor
		for (uint8_t i=0; i<8; i++) {
			channels[i] = 0xFFFF;
		}
		value = next_value = 0;
		idx = 0;
	}

	/*
	   assignment of UDR2 selects which ADC channel
	   to output next
	*/
	ADC_UDR2& operator=(uint8_t cmd) {
		float next_analog;
		uint8_t chan;
		switch (cmd) {
		case 0x87: chan = 0; break;
		case 0xC7: chan = 1; break;
		case 0x97: chan = 2; break;
		case 0xD7: chan = 3; break;
		case 0xA7: chan = 4; break;
		case 0xE7: chan = 5; break;
		case 0xB7: chan = 6; break;
		case 0xF7: chan = 7; break;
		case 0:
		default: return *this;
		}
		next_analog = channels[chan];
		idx = 1;
		if (next_analog > 0xFFF) next_analog = 0xFFF;
		if (next_analog < 0) next_analog = 0;
		next_value = ((unsigned)next_analog) << 3;
		return *this;
	}

	/*
	  read from UDR2 fetches a byte from the channel
	 */
	operator int() {
		uint8_t ret;
		if (idx & 1) {
			ret = (value&0xFF);
			value = next_value;
		} else {
			ret = (value>>8);
		}
		idx ^= 1;
		return ret;
	}

	/*
	  interface to set a channel value from SITL
	 */
	void set(uint8_t i, float v) {
		channels[i] = v;
	}
};

#endif // _SITL_ADC_H
