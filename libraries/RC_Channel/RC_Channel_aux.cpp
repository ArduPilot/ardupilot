// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "RC_Channel_aux.h"

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo RC_Channel_aux::var_info[] = {
    AP_NESTEDGROUPINFO(RC_Channel, 0),

    // @Param: FUNCTION
    // @DisplayName: Servo out function
    // @Description: Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
    // @Values: 0:Disabled,1:RCPassThru,2:Flap,3:Flap_auto,4:Aileron,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,11:release,12:mount2_pan,13:mount2_tilt,14:mount2_roll,15:mount2_open,16:DifferentialSpoiler1,17:DifferentialSpoiler2,18:AileronWithInput,19:Elevator,20:ElevatorWithInput,21:Rudder,24:Flaperon1,25:Flaperon2,26:GroundSteering,27:Parachute,28:EPM,29:LandingGear,30:EngineRunEnable,31:HeliRSC,32:HeliTailRSC,33:Motor1,34:Motor2,35:Motor3,36:Motor4,37:Motor5,38:Motor6,39:Motor7,40:Motor8,51:RCIN1,52:RCIN2,53:RCIN3,54:RCIN4,55:RCIN5,56:RCIN6,57:RCIN7,58:RCIN8,59:RCIN9,60:RCIN10,61:RCIN11,62:RCIN12,63:RCIN13,64:RCIN14,65:RCIN15,66:RCIN16,67:Ignition,68:Choke,69:Starter,70:Throttle
    // @User: Standard
    AP_GROUPINFO("FUNCTION",       1, RC_Channel_aux, function, 0),

    AP_GROUPEND
};

RC_Channel_aux *RC_Channel_aux::_aux_channels[RC_AUX_MAX_CHANNELS];
uint64_t RC_Channel_aux::_function_mask[2];
bool RC_Channel_aux::_initialised;

void
RC_Channel_aux::set_function_mask(uint8_t fn)
{
    uint8_t idx = fn / 64;
    uint8_t bit = fn % 64;
    _function_mask[idx] |= (1ULL<<(uint8_t)bit);
}

void
RC_Channel_aux::clear_function_mask(void)
{
    memset(_function_mask, 0, sizeof(_function_mask));
}

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
        set_radio_out(get_radio_in());
        break;
    case k_rcin1 ... k_rcin16: // rc pass-thru
        set_radio_out(hal.rcin->read(function-k_rcin1));
        break;
    case k_motor1 ... k_motor8:
        // handled by AP_Motors::rc_write()
        return;
    }
    hal.rcout->write(_ch_out, get_radio_out());
}

/*
  call output_ch() on all auxiliary channels
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
  prevent a channel from being used for auxiliary functions
  This is used by the copter code to ensure channels used for motors
  can't be used for auxiliary functions
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

/* 
   setup a channels aux servo function
*/
void RC_Channel_aux::aux_servo_function_setup(void)
{
    switch (function) {
    case RC_Channel_aux::k_flap:
    case RC_Channel_aux::k_flap_auto:
    case RC_Channel_aux::k_egg_drop:
        set_range_out(0,100);
        break;
    case RC_Channel_aux::k_heli_rsc:
    case RC_Channel_aux::k_heli_tail_rsc:
        set_range_out(0,1000);
        break;
    case RC_Channel_aux::k_aileron_with_input:
    case RC_Channel_aux::k_elevator_with_input:
        set_angle(4500);
        break;
    case RC_Channel_aux::k_aileron:
    case RC_Channel_aux::k_elevator:
    case RC_Channel_aux::k_dspoiler1:
    case RC_Channel_aux::k_dspoiler2:
    case RC_Channel_aux::k_rudder:
    case RC_Channel_aux::k_steering:
    case RC_Channel_aux::k_flaperon1:
    case RC_Channel_aux::k_flaperon2:
        set_angle_out(4500);
        break;
    case RC_Channel_aux::k_motor_tilt:
        // tenth percentage tilt
        set_range_out(0,1000);
        break;
    case RC_Channel_aux::k_throttle:
        // fixed wing throttle
        set_range_out(0,100);
        break;
    default:
        break;
    }

    if (function < k_nr_aux_servo_functions) {
        set_function_mask((uint8_t)function.get());
    }
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
    clear_function_mask();

    // set auxiliary ranges
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] == NULL) continue;
        _aux_channels[i]->aux_servo_function_setup();
	}
    _initialised = true;
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
            _aux_channels[i]->set_radio_out(value);
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
        	   int16_t value2 = value - 1500 + _aux_channels[i]->get_radio_trim();
			   _aux_channels[i]->set_radio_out(constrain_int16(value2,_aux_channels[i]->get_radio_min(),_aux_channels[i]->get_radio_max()));
            _aux_channels[i]->output();
		  }
    }
}

/*
  set and save the trim value to radio_in for all channels matching
  the given function type
 */
void
RC_Channel_aux::set_trim_to_radio_in_for(RC_Channel_aux::Aux_servo_function_t function)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			   if (_aux_channels[i]->get_radio_in() != 0) {
				    _aux_channels[i]->set_radio_trim( _aux_channels[i]->get_radio_in());
				    _aux_channels[i]->save_radio_trim();
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
            _aux_channels[i]->set_radio_out( _aux_channels[i]->get_radio_min());
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
            _aux_channels[i]->set_radio_out(_aux_channels[i]->get_radio_max());
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
			   _aux_channels[i]->set_radio_out( _aux_channels[i]->get_radio_trim());
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
			   _aux_channels[i]->set_radio_out(_aux_channels[i]->get_radio_in());
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
RC_Channel_aux::set_servo_out_for(RC_Channel_aux::Aux_servo_function_t function, int16_t value)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function.get() == function) {
			   _aux_channels[i]->set_servo_out(value);
			   _aux_channels[i]->calc_pwm();
            _aux_channels[i]->output();
		  }
    }
}

/*
  setup failsafe value for an auxiliary function type to a LimitValue
 */
void
RC_Channel_aux::set_servo_failsafe_pwm(RC_Channel_aux::Aux_servo_function_t function, uint16_t pwm)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < RC_AUX_MAX_CHANNELS; i++) {
        const RC_Channel_aux *ch = _aux_channels[i];
        if (ch && ch->function.get() == function) {
            hal.rcout->set_failsafe_pwm(1U<<ch->get_ch_out(), pwm);
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
            ch->set_radio_out(pwm);
            if (ch->function.get() == k_manual) {
                // in order for output_ch() to work for k_manual we
                // also have to override radio_in
                ch->set_radio_in(pwm);
            }
            if (ch->function.get() >= k_rcin1 && ch->function.get() <= k_rcin16) {
                // save for k_rcin*
                ch->set_radio_in(pwm);
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
        uint8_t fn = (uint8_t)function;
        uint8_t idx = fn / 64;
        uint8_t bit = fn % 64;
        return (_function_mask[idx] & (1ULL<<bit)) != 0;
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
			_aux_channels[i]->set_servo_out(value);
			_aux_channels[i]->set_range(angle_min, angle_max);
			_aux_channels[i]->calc_pwm();
			_aux_channels[i]->output();
		}
	}
}

/*
  set the default channel an auxiliary output function should be on
 */
bool RC_Channel_aux::set_aux_channel_default(RC_Channel_aux::Aux_servo_function_t function, uint8_t channel)
{
    if (!_initialised) {
        update_aux_servo_function();
    }
    if (function_assigned(function)) {
        // already assigned
        return true;
    }
    for (uint8_t i=0; i<RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->_ch_out == channel) {
            if (_aux_channels[i]->function != k_none) {
                if (_aux_channels[i]->function == function) {
                    return true;
                }
                hal.console->printf("Channel %u already assigned %u\n",
                                    (unsigned)channel,
                                    (unsigned)_aux_channels[i]->function);
                return false;
            }
            _aux_channels[i]->function.set(function);
            _aux_channels[i]->aux_servo_function_setup();
            return true;
        }
    }
    hal.console->printf("AUX channel %u not available\n",
                        (unsigned)channel);
    return false;    
}

// find first channel that a function is assigned to
bool RC_Channel_aux::find_channel(RC_Channel_aux::Aux_servo_function_t function, uint8_t &chan)
{
    if (!_initialised) {
        update_aux_servo_function();
    }
    if (!function_assigned(function)) {
        return false;
    }
    for (uint8_t i=0; i<RC_AUX_MAX_CHANNELS; i++) {
        if (_aux_channels[i] && _aux_channels[i]->function == function) {
            chan = _aux_channels[i]->_ch_out;
            return true;
        }
    }
    return false;
}
