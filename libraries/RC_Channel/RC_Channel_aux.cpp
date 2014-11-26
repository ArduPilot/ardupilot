// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "RC_Channel_aux.h"

#include <AP_Math.h>
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo RC_Channel_aux::var_info[] PROGMEM = {
    AP_NESTEDGROUPINFO(RC_Channel, 0),

    // @Param: FUNCTION
    // @DisplayName: Servo out function
    // @Description: Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
    // @Values: 0:Disabled,1:RCPassThru,2:Flap,3:Flap_auto,4:Aileron,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,11:release,12:mount2_pan,13:mount2_tilt,14:mount2_roll,15:mount2_open,16:DifferentialSpoiler1,17:DifferentialSpoiler2,18:AileronWithInput,19:Elevator,20:ElevatorWithInput,21:Rudder,24:Flaperon1,25:Flaperon2,26:GroundSteering,27:Parachute
    // @User: Standard
    AP_GROUPINFO("FUNCTION",       1, RC_Channel_aux, function, 0),

    AP_GROUPEND
};

RC_Channel_aux *RC_Channel_aux::_aux_channels[RC_AUX_MAX_CHANNELS];
uint32_t RC_Channel_aux::_function_mask;

/// map a function to a servo channel and output it
void
RC_Channel_aux::output_ch(void)
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
    hal.rcout->write(_ch_out, radio_out);
}

/*
  call output_ch() on all auxillary channels
 */
void
RC_Channel_aux::output_ch_all(void)
{
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i]) {
            _aux_channels[i]->output_ch();
        }
    }    
}

/*
  prevent a channel from being used for auxillary functions
  This is used by the copter code to ensure channels used for motors
  can't be used for auxillary functions
*/
void RC_Channel_aux::disable_aux_channel(uint8_t channel)
{
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->_ch_out == channel) {
            _aux_channels[i] = NULL;
        }
    }    
}

/*
  return the current function for a channel
*/
RC_Channel_aux::Aux_servo_function_t RC_Channel_aux::channel_function(uint8_t channel)
{
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->_ch_out == channel) {
            return (RC_Channel_aux::Aux_servo_function_t)_aux_channels[i]->function.get();
        }
    }    
    return RC_Channel_aux::k_none;
}

/// Update the _aux_channels array of pointers to rc_x channels
/// This is to be done before rc_init so that the channels get correctly initialized.
/// It also should be called periodically because the user might change the configuration and
/// expects the changes to take effect instantly
/// Supports up to eight aux servo outputs (typically CH5 ... CH11)
/// All servos must be configured with a single call to this function
/// (do not call this twice with different parameters, the second call will reset the effect of the first call)
void RC_Channel_aux::update_aux_servo_function(void)
{
    // set auxiliary ranges
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] == NULL) continue;
		RC_Channel_aux::Aux_servo_function_t function = (RC_Channel_aux::Aux_servo_function_t)_aux_channels[i]->function.get();
		switch (function) {
		case RC_Channel_aux::k_flap:
		case RC_Channel_aux::k_flap_auto:
		case RC_Channel_aux::k_egg_drop:
			_aux_channels[i]->set_range(0,100);
			break;
		case RC_Channel_aux::k_aileron:
		case RC_Channel_aux::k_aileron_with_input:
		case RC_Channel_aux::k_elevator:
		case RC_Channel_aux::k_elevator_with_input:
		case RC_Channel_aux::k_dspoiler1:
		case RC_Channel_aux::k_dspoiler2:
		case RC_Channel_aux::k_rudder:
		case RC_Channel_aux::k_steering:
		case RC_Channel_aux::k_flaperon1:
		case RC_Channel_aux::k_flaperon2:
		    _aux_channels[i]->set_angle(4500);
			break;
		default:
			break;
		}
	}

    // create a function mask to make updates master
    _function_mask = 0;
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i]) {
            RC_Channel_aux::Aux_servo_function_t function = (RC_Channel_aux::Aux_servo_function_t)_aux_channels[i]->function.get();
            if (function < k_nr_aux_servo_functions) {
                _function_mask |= (1UL<<(uint8_t)function);
            }
		}
	}
}

/// Should be called after the the servo functions have been initialized
void RC_Channel_aux::enable_aux_servos()
{
    update_aux_servo_function();

    // enable all channels that are not set to a valid function. This
    // includes k_none servos, which allows those to get their initial
    // trim value on startup
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
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
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			_aux_channels[i]->radio_out = constrain_int16(value,_aux_channels[i]->radio_min,_aux_channels[i]->radio_max);
            _aux_channels[i]->output();
		}
    }
}

/*
  set radio_out for all channels matching the given function type, allow radio_trim to center servo
 */
void
RC_Channel_aux::set_radio_trimmed(RC_Channel_aux::Aux_servo_function_t function, int16_t value)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
        	int16_t value2 = value - 1500 + _aux_channels[i]->radio_trim;
			_aux_channels[i]->radio_out = constrain_int16(value2,_aux_channels[i]->radio_min,_aux_channels[i]->radio_max);
            _aux_channels[i]->output();
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
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			if (_aux_channels[i]->radio_in != 0) {
				_aux_channels[i]->radio_trim = _aux_channels[i]->radio_in;
				_aux_channels[i]->radio_trim.save();
			}
		}
    }
}

/*
  set the radio_out value for any channel with the given function to radio_min
 */
void
RC_Channel_aux::set_radio_to_min(RC_Channel_aux::Aux_servo_function_t function)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
            _aux_channels[i]->radio_out = _aux_channels[i]->radio_min;
            _aux_channels[i]->output();
		}
    }
}

/*
  set the radio_out value for any channel with the given function to radio_max
 */
void
RC_Channel_aux::set_radio_to_max(RC_Channel_aux::Aux_servo_function_t function)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
            _aux_channels[i]->radio_out = _aux_channels[i]->radio_max;
            _aux_channels[i]->output();
		}
    }
}

/*
  set the radio_out value for any channel with the given function to radio_trim
 */
void
RC_Channel_aux::set_radio_to_trim(RC_Channel_aux::Aux_servo_function_t function)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			_aux_channels[i]->radio_out = _aux_channels[i]->radio_trim;
            _aux_channels[i]->output();
		}
    }
}

/*
  copy radio_in to radio_out for a given function
 */
void
RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::Aux_servo_function_t function, bool do_input_output)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
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
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			_aux_channels[i]->servo_out = value;
			_aux_channels[i]->calc_pwm();
            _aux_channels[i]->output();
		}
    }
}

/*
  setup failsafe value for an auxiliary function type to a LimitValue
 */
void
RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::Aux_servo_function_t function, RC_Channel::LimitValue limit)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        const RC_Channel_aux *ch = _aux_channels[i];
        if (ch && ch->function.get() == function) {
            uint16_t pwm = ch->get_limit_pwm(limit);
            hal.rcout->set_failsafe_pwm(1U<<ch->get_ch_out(), pwm);
        }
    }
}

/*
  set radio output value for an auxiliary function type to a LimitValue
 */
void
RC_Channel_aux::set_servo_limit(RC_Channel_aux::Aux_servo_function_t function, RC_Channel::LimitValue limit)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        RC_Channel_aux *ch = _aux_channels[i];
        if (ch && ch->function.get() == function) {
            uint16_t pwm = ch->get_limit_pwm(limit);
            ch->radio_out = pwm;
            if (ch->function.get() == k_manual) {
                // in order for output_ch() to work for k_manual we
                // also have to override radio_in
                ch->radio_in = pwm;
            }
        }
    }
}

/*
  return true if a particular function is assigned to at least one RC channel
 */
bool
RC_Channel_aux::function_assigned(RC_Channel_aux::Aux_servo_function_t function)
{
    if (function < k_nr_aux_servo_functions) {
        return (_function_mask & (1UL<<function)) != 0;
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
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			_aux_channels[i]->servo_out = value;
			_aux_channels[i]->set_range(angle_min, angle_max);
			_aux_channels[i]->calc_pwm();
			_aux_channels[i]->output();
		}
	}
}
