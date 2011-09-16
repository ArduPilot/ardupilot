// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_RC_Channel.h
/// @brief	AP_RC_Channel manager, with EEPROM-backed storage of constants.

#ifndef AP_RC_Channel_h
#define AP_RC_Channel_h

//#include <AP_Common.h>
#include <stdint.h>

/// @class	AP_RC_Channel
/// @brief	Object managing one RC channel
class AP_RC_Channel{
  public:
	/// Constructor
	///
	/// @param key      EEPROM storage key for the channel trim parameters.
	/// @param name     Optional name for the group.
	///
	AP_RC_Channel(uint16_t address) :
		_address(address),
		_high(1),
		_filter(true),
		_reverse(1),
		dead_zone(0){}

	AP_RC_Channel() :
		_high(1),
		_filter(true),
		_reverse(1),
		dead_zone(0){}

	// setup min and max radio values in CLI
	void 		update_min_max();
	void 		zero_min_max();

	// startup
	void 		load_eeprom(void);
	void 		save_eeprom(void);
	void 		save_trim(void);
	void 		load_trim(void);
	void		set_filter(bool filter);

	// setup the control preferences
	void 		set_range(int low, int high);
	void 		set_angle(int angle);
	void 		set_reverse(bool reverse);
	bool		get_reverse(void);

	// read input from APM_RC - create a control_in value
	void 		set_pwm(int pwm);

	// pwm is stored here
	int16_t		radio_in;

	// call after first set_pwm
	void 		trim();

	// did our read come in 50µs below the min?
	bool		get_failsafe(void);

	// value generated from PWM
	int16_t 	control_in;
	int16_t 	dead_zone; // used to keep noise down and create a dead zone.

	int			control_mix(float value);

	// current values to the servos - degrees * 100 (approx assuming servo is -45 to 45 degrees except [3] is 0 to 100
	int16_t 	servo_out;

	// generate PWM from servo_out value
	void 		calc_pwm(void);

	// PWM is without the offset from radio_min
	int16_t 	pwm_out;
	int16_t 	radio_out;

	int16_t		radio_min;
	int16_t		radio_trim;
	int16_t		radio_max;

	// includes offset from PWM
	//int16_t 	get_radio_out(void);

	int16_t		pwm_to_angle();
	float		norm_input();
	float		norm_output();
	int16_t		angle_to_pwm();
	int16_t		pwm_to_range();
	int16_t		range_to_pwm();

  private:
	bool		_filter;
	int8_t 		_reverse;
	int16_t		_address;
	bool 		_type;
	int16_t 	_high;
	int16_t 	_low;
};

#endif




