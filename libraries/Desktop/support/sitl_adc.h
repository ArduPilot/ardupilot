/*
  ADS7844 register emulation
  Code by Andrew Tridgell November 2011
 */

#ifndef _SITL_ADC_H
#define _SITL_ADC_H

#include <stdlib.h>
#include <math.h>

static const float vibration_level = 0.2;
static const float drift_speed = 0.2; // degrees/second/minute
static const float drift_time  = 5; // time to reverse drift direction (minutes)
// order                              zgyro, xgyro, ygyro, temp, xacc, yacc, zacc, aspd
static const float noise_scale[8] = {   150,   150,   150,    0, 400,  400,  400,    0 };
static const float noise_offset[8]= {     0,     0,     0,    0,   0,    0,    0,    0 };
static const float drift_rate[8]  = {     1.0, 1.0,   1.0,    0,   0,    0,    0,    0 };
static const float base_noise = 2;

// generate a random float between -1 and 1
static double rand_float(void)
{
	float ret = ((unsigned)random()) % 2000000;
	return (ret - 1.0e6) / 1.0e6;
}

static inline float gyro_drift(uint8_t chan)
{
	if (drift_rate[chan] * drift_speed == 0.0) {
		return 0;
	}
	extern long unsigned int micros(void);
	double period  = drift_rate[chan] * drift_time * 2;
	double minutes = fmod(micros() / 60.0e6, period);
	if (minutes < period/2) {
		return minutes * drift_speed / 0.4;
	}
	return (period - minutes) * drift_speed / 0.4;

}

static inline float noise_generator(uint8_t chan)
{
	extern float sitl_motor_speed[4];
	uint8_t i;
	float noise = 0;
	uint8_t noise_count=0;
	extern long unsigned int micros(void);
	for (i=0; i<4; i++) {
		if (sitl_motor_speed[i] > 0.0) {
			float n = rand_float() * noise_scale[chan] * vibration_level;
			//double t = micros() / 1.0e6;
			//float freq = (rand_float() + 1.0) * sitl_motor_speed[i];
			//noise += sin(fmod(t * freq * 2 * M_PI + i,
			//2*M_PI)) * n;
			noise += n + noise_offset[chan];
			noise_count++;
		}
	}
	if (noise_count == 0) {
		return gyro_drift(chan) + rand_float() * base_noise * vibration_level;
	}
	return gyro_drift(chan) + noise/noise_count;
}

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
		next_analog += noise_generator(chan) + 0.5;
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
