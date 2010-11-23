/*
	RC_Channel library.
	Code by Jason Short. 2010 
	DIYDrones.com
*/

#ifndef RC_Channel_h
#define RC_Channel_h

#include <inttypes.h>
#include "WProgram.h"

class RC_Channel
{
  public:
	RC_Channel();
	
	// startup
	void 		load_eeprom(int address);
	void 		save_eeprom(int address);		
	void 		set_radio_range(int r_min, int r_max);

	// setup the control preferences
	void 		set_range(int high, int low);
	void 		set_angle(int angle);

	// call after first read
	void 		set_trim(int pwm);

	// read input from APM_RC - create a control_in value
	void 		set_pwm(int pwm);
	
	// did our read come in 50µs below the min?
	boolean		get_failsafe(void);
	
	// value generated from PWM
	int16_t 	control_in;

	// current values to the servos - degrees * 100 (approx assuming servo is -45 to 45 degrees except [3] is 0 to 100
	int16_t 	servo_out;

	// generate PWM from servo_out value
	void 		calc_pwm(void);

	// PWM is without the offset from radio_min
	int16_t 	pwm_out;
	int16_t 	radio_out;
	
	// includes offset from PWM
	//int16_t 	get_radio_out(void);
	
  private:
	int16_t		channel_filter();
	int16_t		pwm_to_angle();
	int16_t		angle_to_pwm();
	int16_t		pwm_to_range();
	int16_t		range_to_pwm();

	byte 		_channel;
	boolean 	_type;
	boolean 	_filter;

	int16_t		_radio_in;
	int16_t 	_radio_min;
	int16_t 	_radio_trim;
	int16_t 	_radio_max;
		
	int16_t 	_high;
	int16_t 	_low;
};

#endif	




