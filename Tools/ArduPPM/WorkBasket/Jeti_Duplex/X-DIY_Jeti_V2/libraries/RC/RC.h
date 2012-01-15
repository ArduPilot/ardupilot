#ifndef RC_h
#define RC_h

#include <inttypes.h>
#include "WProgram.h"

#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3
#define CH5 4
#define CH6 5
#define CH7 6
#define CH8 7

#define MIN_PULSEWIDTH 900
#define MAX_PULSEWIDTH 2100

#define ELEVONS 1

class RC
{
  public:
  //	RC();
	virtual void init();
	virtual void trim();
	virtual void read();
	virtual void output();
	virtual void set_channel_direction(uint8_t ch, int8_t dir);
	virtual void set_ch_pwm(uint8_t ch, uint16_t pwm);
	virtual void twitch_servos(void);

	void set_failsafe(uint16_t fs);
	void set_mix_mode(uint8_t mode);
	
	uint8_t failsafe;
	
  protected:
	void check_throttle_failsafe(uint16_t throttle);
  	uint8_t 	_fs_counter;
  	uint8_t 	_mix_mode;	// 0 = normal, 1 = elevons
  	uint8_t 	_direction_mask;
  	uint16_t 	_fs_value;  // PWM value to trigger failsafe flag
};

#endif
