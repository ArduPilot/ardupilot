// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <APM_RC.h>
#include "RC_Channel_aux.h"

const AP_Param::GroupInfo RC_Channel_aux::var_info[] PROGMEM = {
    AP_NESTEDGROUPINFO(RC_Channel, 0),

    // @Param: FUNCTION
    // @DisplayName: Servo out function
    // @Description: Setting this to Disabled(0) will disable this output, any other value will enable the corresponding function
    // @Values: 0:Disabled,1:Manual,2:Flap,3:Flap_auto,4:Aileron,5:flaperon,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,11:release,12:mount2_pan,13:mount2_tilt,14:mount2_roll,15:mount2_open,16:DifferentialSpoiler1,17:DifferentialSpoiler2,18:AileronWithInput
    // @User: Standard
    AP_GROUPINFO("FUNCTION",       1, RC_Channel_aux, function, 0),

    AP_GROUPEND
};

static RC_Channel_aux *_aux_channels[7];

/// map a function to a servo channel and output it
void
RC_Channel_aux::output_ch(unsigned char ch_nr)
{
    // take care of two corner cases
    switch(function)
    {
    case k_none:                // disabled
        return;
    case k_manual:              // manual
        radio_out = radio_in;
        break;
    }
    _apm_rc->OutputCh(ch_nr, radio_out);
}

/// Update the _aux_channels array of pointers to rc_x channels
/// This is to be done before rc_init so that the channels get correctly initialized.
/// It also should be called periodically because the user might change the configuration and
/// expects the changes to take effect instantly
/// Supports up to seven aux servo outputs (typically CH5 ... CH11)
/// All servos must be configured with a single call to this function
/// (do not call this twice with different parameters, the second call will reset the effect of the first call)
void update_aux_servo_function( RC_Channel_aux* rc_a,
                                RC_Channel_aux* rc_b,
                                RC_Channel_aux* rc_c,
                                RC_Channel_aux* rc_d,
                                RC_Channel_aux* rc_e,
                                RC_Channel_aux* rc_f,
                                RC_Channel_aux* rc_g)
{
	_aux_channels[0] = rc_a;
	_aux_channels[1] = rc_b;
	_aux_channels[2] = rc_c;
	_aux_channels[3] = rc_d;
	_aux_channels[4] = rc_e;
	_aux_channels[5] = rc_f;
	_aux_channels[6] = rc_g;

    // set auxiliary ranges
    for (uint8_t i = 0; i < 7; i++) {
        if (_aux_channels[i] == NULL) continue;
		RC_Channel_aux::Aux_servo_function_t function = (RC_Channel_aux::Aux_servo_function_t)_aux_channels[i]->function.get();
		switch (function) {
		case RC_Channel_aux::k_flap:
		case RC_Channel_aux::k_flap_auto:
		case RC_Channel_aux::k_flaperon:
		case RC_Channel_aux::k_egg_drop:
			_aux_channels[i]->set_range(0,100);
			break;
		case RC_Channel_aux::k_aileron:
		case RC_Channel_aux::k_aileron_with_input:
			_aux_channels[i]->set_angle(4500);
			break;
		case RC_Channel_aux::k_dspoiler1:
		    _aux_channels[i]->set_angle(4500);
			break;
		case RC_Channel_aux::k_dspoiler2:
		    _aux_channels[i]->set_angle(4500);
			break;
		default:
			break;
		}
	}
}

/// Should be called after the the servo functions have been initialized
void
enable_aux_servos()
{
    // enable all channels that are not set to k_none or k_nr_aux_servo_functions
    for (uint8_t i = 0; i < 7; i++) {
        if (_aux_channels[i]) {
			RC_Channel_aux::Aux_servo_function_t function = (RC_Channel_aux::Aux_servo_function_t)_aux_channels[i]->function.get();
			// see if it is a valid function
			if (function < RC_Channel_aux::k_nr_aux_servo_functions) {
				_aux_channels[i]->enable_out();
			}
		}
	}
}

/*
  set radio_out for all channels matching the given function type
 */
void
RC_Channel_aux::set_radio(RC_Channel_aux::Aux_servo_function_t function, int16_t value)
{
    for (uint8_t i = 0; i < 7; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			_aux_channels[i]->radio_out = value;
		}
    }
}


/*
  set and save the trim value to radio_in for all channels matching
  the given function type
 */
void
RC_Channel_aux::set_radio_trim(RC_Channel_aux::Aux_servo_function_t function)
{
    for (uint8_t i = 0; i < 7; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			_aux_channels[i]->radio_trim = _aux_channels[i]->radio_in;
			_aux_channels[i]->radio_trim.save();
		}
    }
}

/*
  set the radio_out value for any channel with the given function to radio_min
 */
void
RC_Channel_aux::set_radio_to_min(RC_Channel_aux::Aux_servo_function_t function)
{
    for (uint8_t i = 0; i < 7; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			_aux_channels[i]->radio_out = _aux_channels[i]->radio_min;
		}
    }
}

/*
  set the radio_out value for any channel with the given function to radio_max
 */
void
RC_Channel_aux::set_radio_to_max(RC_Channel_aux::Aux_servo_function_t function)
{
    for (uint8_t i = 0; i < 7; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			_aux_channels[i]->radio_out = _aux_channels[i]->radio_max;
		}
    }
}

/*
  set the radio_out value for any channel with the given function to radio_trim
 */
void
RC_Channel_aux::set_radio_to_trim(RC_Channel_aux::Aux_servo_function_t function)
{
    for (uint8_t i = 0; i < 7; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			_aux_channels[i]->radio_out = _aux_channels[i]->radio_trim;
		}
    }
}

/*
  copy radio_in to radio_out for a given function
 */
void
RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::Aux_servo_function_t function, bool do_input_output)
{
    for (uint8_t i = 0; i < 7; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			if (do_input_output) {
				_aux_channels[i]->input();
			}
			_aux_channels[i]->radio_out = _aux_channels[i]->radio_in;
			if (do_input_output) {
				_aux_channels[i]->output();
			}
		}
    }
}

/*
  set servo_out and call calc_pwm() for a given function
 */
void
RC_Channel_aux::set_servo_out(RC_Channel_aux::Aux_servo_function_t function, int16_t value)
{
    for (uint8_t i = 0; i < 7; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			_aux_channels[i]->servo_out = value;
			_aux_channels[i]->calc_pwm();
		}
    }
}

/*
  return true if a particular function is assigned to at least one RC channel
 */
bool
RC_Channel_aux::function_assigned(RC_Channel_aux::Aux_servo_function_t function)
{
    for (uint8_t i = 0; i < 7; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			return true;
		}
	}
	return false;
}

/*
  set servo_out and angle_min/max, then calc_pwm and output a
  value. This is used to move a AP_Mount servo
 */
void
RC_Channel_aux::move_servo(RC_Channel_aux::Aux_servo_function_t function,
						   int16_t value, int16_t angle_min, int16_t angle_max)
{
    for (uint8_t i = 0; i < 7; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			_aux_channels[i]->servo_out = value;
			_aux_channels[i]->set_range(angle_min, angle_max);
			_aux_channels[i]->calc_pwm();
			_aux_channels[i]->output();
		}
	}
}
