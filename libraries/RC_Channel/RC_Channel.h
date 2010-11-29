// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	RC_Channel.h
/// @brief	RC_Channel manager, with EEPROM-backed storage of constants.

#ifndef RC_Channel_h
#define RC_Channel_h

#include <stdint.h>

/// @class	RC_Channel
/// @brief	Object managing one RC channel
class RC_Channel{
  public:	
	/// Constructor
	///
	/// A RC_Channel constructed in this fashion does not support save/restore.
	///
	RC_Channel() :
		_address(0)
	{}

	/// Constructor
	///
	/// @param address	EEPROM base address at which RC_Channel parameters
	///					are stored.  Zero if the RC_Channel does not support
	///					save/restore.
	///
	RC_Channel(uint16_t address) :
		_address(address),
		_high(1),
		_filter(true)
	{}

	// setup min and max radio values in CLI
	void 		update_min_max();
	
	// startup
	void 		load_eeprom(void);
	void 		save_eeprom(void);		
	void 		save_trim(void);		
	void		set_filter(bool filter);

	// setup the control preferences
	void 		set_range(int low, int high);
	void 		set_angle(int angle);

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

	int16_t 	radio_min;
	int16_t 	radio_trim;
	int16_t 	radio_max;
	
	// includes offset from PWM
	//int16_t 	get_radio_out(void);
	
  private:
	bool		_filter;
	int16_t		pwm_to_angle();
	int16_t		angle_to_pwm();
	int16_t		pwm_to_range();
	int16_t		range_to_pwm();

	int16_t		_address;			///< EEPROM address for save/restore of P/I/D
	bool 		_type;				
	int16_t 	_high;
	int16_t 	_low;
};

#endif	




