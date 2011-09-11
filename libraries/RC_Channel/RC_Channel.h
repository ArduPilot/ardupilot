// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	RC_Channel.h
/// @brief	RC_Channel manager, with EEPROM-backed storage of constants.

#ifndef RC_Channel_h
#define RC_Channel_h

#include <AP_Common.h>
// TODO is this include really necessary ?
#include <stdint.h>

/// @class	RC_Channel
/// @brief	Object managing one RC channel
class RC_Channel{
  protected:
	AP_Var_group    _group;		// must be before all vars to keep ctor init order correct

  public:
	/// Constructor
	///
	/// @param key      EEPROM storage key for the channel trim parameters.
	/// @param name     Optional name for the group.
	///
	RC_Channel(AP_Var::Key key, const prog_char_t *name) :
	    _group(key, name),
        radio_min (&_group, 0, 1500, name ? PSTR("MIN") : 0), // suppress name if group has no name
        radio_trim(&_group, 1, 1500, name ? PSTR("TRIM") : 0),
        radio_max (&_group, 2, 1500, name ? PSTR("MAX") : 0),
		_high(1),
		_filter(true),
		_reverse  (&_group, 3,    1, name ? PSTR("REV") : 0),
		dead_zone(0),
		scale_output(1.0)
	{}

	// setup min and max radio values in CLI
	void 		update_min_max();
	void 		zero_min_max();

	// startup
	void 		load_eeprom(void);
	void 		save_eeprom(void);
	void 		save_trim(void);
	void		set_filter(bool filter);
	void		set_type(uint8_t t);

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

	AP_Int16 	radio_min;
	AP_Int16 	radio_trim;
	AP_Int16 	radio_max;

	// includes offset from PWM
	//int16_t 	get_radio_out(void);

	int16_t		pwm_to_angle();
	float		norm_input();
	float		norm_output();
	int16_t		angle_to_pwm();
	int16_t		pwm_to_range();
	int16_t		range_to_pwm();

	float		scale_output;

  private:
	bool		_filter;
	AP_Int8 	_reverse;

	uint8_t 	_type;
	int16_t 	_high;
	int16_t 	_low;
};

/// @class	RC_Channel_aux
/// @brief	Object managing one aux. RC channel (CH5-8), with information about its function
/// 	    Also contains physical min,max angular deflection, to allow calibrating open-loop servo movements
class RC_Channel_aux : public RC_Channel{
public:
	/// Constructor
	///
	/// @param key      EEPROM storage key for the channel trim parameters.
	/// @param name     Optional name for the group.
	///
	RC_Channel_aux(AP_Var::Key key, const prog_char_t *name) :
		RC_Channel(key, name),
		function  (&_group, 4,  k_none, name ? PSTR("FUNCTION")  : 0), // suppress name if group has no name
		angle_min (&_group, 5, -4500, name ? PSTR("ANGLE_MIN") : 0), // assume -45 degrees min deflection
		angle_max (&_group, 6,  4500, name ? PSTR("ANGLE_MAX") : 0)  // assume  45 degrees max deflection
	{}

	typedef enum
	{
		k_none			= 0,	// disabled
		k_manual		= 1,	// manual, just pass-thru the RC in signal
		k_flap			= 2,	// flap
		k_flap_auto		= 3,	// flap automated
		k_aileron		= 4,	// aileron
		k_flaperon		= 5,	// flaperon (flaps and aileron combined, needs two independent servos one for each wing)
		k_mount_yaw		= 6,	// mount yaw (pan)
		k_mount_pitch	= 7,	// mount pitch (tilt)
		k_mount_roll	= 8,	// mount roll
		k_cam_trigger	= 9,	// camera trigger
		k_cam_open		= 10,	// camera open
		k_egg_drop		= 11,	// egg drop
		k_nr_aux_servo_functions // This must be the last enum value (only add new values _before_ this one)
	} Aux_servo_function_t;

	// TODO It would be great if the "packed" attribute could be added to this somehow
	// It would probably save some memory. But it can only be added to enums and not to typedefs :(
	/*AP_VARDEF(Aux_servo_function_t, Aux_srv_func);    // defines AP_Aux_srv_func

	AP_Aux_srv_func 	function;*/	// 0=disabled, 1=manual, 2=flap, 3=flap auto, 4=aileron, 5=flaperon, 6=mount yaw (pan), 7=mount pitch (tilt), 8=mount roll, 9=camera trigger, 10=camera open, 11=egg drop
	AP_Int8 	function;	// 0=disabled, 1=manual, 2=flap, 3=flap auto, 4=aileron, 5=flaperon, 6=mount yaw (pan), 7=mount pitch (tilt), 8=mount roll, 9=camera trigger, 10=camera open, 11=egg drop
	AP_Int16 	angle_min;	// min angle limit of actuated surface in 0.01 degree units
	AP_Int16 	angle_max;	// max angle limit of actuated surface in 0.01 degree units

	int16_t closest_limit(int16_t angle);	// saturate to the closest angle limit if outside of min max angle interval

	void output_ch(unsigned char ch_nr);	// map a function to a servo channel and output it

};

#endif




