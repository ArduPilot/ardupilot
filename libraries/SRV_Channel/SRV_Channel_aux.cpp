/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  SRV_Channel_aux.cpp - handling of servo auxillary functions
 */
#include "SRV_Channel.h"

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_RCMapper/AP_RCMapper.h>

extern const AP_HAL::HAL& hal;

/// map a function to a servo channel and output it
void SRV_Channel::output_ch(void)
{
    int8_t passthrough_from = -1;

    // take care of special function cases
    switch(function)
    {
    case k_manual:              // manual
        passthrough_from = ch_num;
        break;
    case k_rcin1 ... k_rcin16: // rc pass-thru
        passthrough_from = int8_t(function - k_rcin1);
        break;
    case k_motor1 ... k_motor8:
        // handled by AP_Motors::rc_write()
        return;
    }
    if (passthrough_from != -1) {
        // we are doing passthrough from input to output for this channel
        RC_Channel *rc = RC_Channels::rc_channel(passthrough_from);
        if (rc) {
            if (SRV_Channels::passthrough_disabled()) {
                output_pwm = rc->get_radio_trim();
            } else {
                output_pwm = rc->get_radio_in();
            }
        }
    }
    hal.rcout->write(ch_num, output_pwm);
}

/*
  call output_ch() on all channels
 */
void SRV_Channels::output_ch_all(void)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        channels[i].output_ch();
    }
}

/*
  return the current function for a channel
*/
SRV_Channel::Aux_servo_function_t SRV_Channels::channel_function(uint8_t channel)
{
    if (channel < NUM_SERVO_CHANNELS) {
        return (SRV_Channel::Aux_servo_function_t)channels[channel].function.get();
    }
    return SRV_Channel::k_none;
}

/*
   setup a channels aux servo function
*/
void SRV_Channel::aux_servo_function_setup(void)
{
    if (type_setup) {
        return;
    }
    switch (function) {
    case k_flap:
    case k_flap_auto:
    case k_egg_drop:
        set_range(100);
        break;
    case k_heli_rsc:
    case k_heli_tail_rsc:
    case k_motor_tilt:
        set_range(1000);
        break;
    case k_aileron_with_input:
    case k_elevator_with_input:
    case k_aileron:
    case k_elevator:
    case k_dspoiler1:
    case k_dspoiler2:
    case k_rudder:
    case k_steering:
    case k_flaperon1:
    case k_flaperon2:
    case k_tiltMotorLeft:
    case k_tiltMotorRight:
        set_angle(4500);
        break;
    case k_throttle:
    case k_throttleLeft:
    case k_throttleRight:
        // fixed wing throttle
        set_range(100);
        break;
    default:
        break;
    }
}

/// setup the output range types of all functions
void SRV_Channels::update_aux_servo_function(void)
{
    function_mask.clearall();

    for (uint8_t i = 0; i < SRV_Channel::k_nr_aux_servo_functions; i++) {
        functions[i].channel_mask = 0;
    }
    
    // set auxiliary ranges
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        channels[i].aux_servo_function_setup();
        function_mask.set((uint8_t)channels[i].function.get());
        functions[channels[i].function.get()].channel_mask |= 1U<<i;
    }
    initialised = true;
}


/// Should be called after the the servo functions have been initialized
void SRV_Channels::enable_aux_servos()
{
    update_aux_servo_function();

    // enable all channels that are set to a valid function. This
    // includes k_none servos, which allows those to get their initial
    // trim value on startup
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        SRV_Channel::Aux_servo_function_t function = (SRV_Channel::Aux_servo_function_t)channels[i].function.get();
        // see if it is a valid function
        if (function < SRV_Channel::k_nr_aux_servo_functions) {
            hal.rcout->enable_ch(channels[i].ch_num);
        }
    }
}

/*
  set radio_out for all channels matching the given function type
 */
void SRV_Channels::set_output_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t value)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function.get() == function) {
            channels[i].set_output_pwm(value);
            channels[i].output_ch();
        }
    }
}

/*
  set radio_out for all channels matching the given function type
  trim the output assuming a 1500 center on the given value
 */
void
SRV_Channels::set_output_pwm_trimmed(SRV_Channel::Aux_servo_function_t function, int16_t value)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function.get() == function) {
            int16_t value2 = value - 1500 + channels[i].get_trim();
            channels[i].set_output_pwm(constrain_int16(value2,channels[i].get_output_min(),channels[i].get_output_max()));
            channels[i].output_ch();
          }
    }
}

/*
  set and save the trim value to radio_in for all channels matching
  the given function type
 */
void
SRV_Channels::set_trim_to_radio_in_for(SRV_Channel::Aux_servo_function_t function)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function.get() == function) {
            RC_Channel *rc = RC_Channels::rc_channel(channels[i].ch_num);
            if (rc && rc->get_radio_in() != 0) {
                rc->set_radio_trim(rc->get_radio_in());
                rc->save_radio_trim();
            }
        }
    }
}

/*
  copy radio_in to radio_out for a given function
 */
void
SRV_Channels::copy_radio_in_out(SRV_Channel::Aux_servo_function_t function, bool do_input_output)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function.get() == function) {
            RC_Channel *rc = RC_Channels::rc_channel(channels[i].ch_num);
            if (rc == nullptr) {
                continue;
            }
            if (do_input_output) {
                rc->read();
            }
            channels[i].set_output_pwm(rc->get_radio_in());
            if (do_input_output) {
                channels[i].output_ch();
            }
        }
    }
}

/*
  setup failsafe value for an auxiliary function type to a LimitValue
 */
void
SRV_Channels::set_failsafe_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t pwm)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const SRV_Channel &ch = channels[i];
        if (ch.function.get() == function) {
            hal.rcout->set_failsafe_pwm(1U<<ch.ch_num, pwm);
        }
    }
}

/*
  setup failsafe value for an auxiliary function type to a LimitValue
 */
void
SRV_Channels::set_failsafe_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::LimitValue limit)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const SRV_Channel &ch = channels[i];
        if (ch.function.get() == function) {
            uint16_t pwm = ch.get_limit_pwm(limit);
            hal.rcout->set_failsafe_pwm(1U<<ch.ch_num, pwm);
        }
    }
}

/*
  setup safety value for an auxiliary function type to a LimitValue
 */
void
SRV_Channels::set_safety_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::LimitValue limit)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const SRV_Channel &ch = channels[i];
        if (ch.function.get() == function) {
            uint16_t pwm = ch.get_limit_pwm(limit);
            hal.rcout->set_safety_pwm(1U<<ch.ch_num, pwm);
        }
    }
}

/*
  set radio output value for an auxiliary function type to a LimitValue
 */
void
SRV_Channels::set_output_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::LimitValue limit)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &ch = channels[i];
        if (ch.function.get() == function) {
            uint16_t pwm = ch.get_limit_pwm(limit);
            ch.set_output_pwm(pwm);
            if (ch.function.get() == SRV_Channel::k_manual) {
                RC_Channel *rc = RC_Channels::rc_channel(ch.ch_num);
                if (rc != nullptr) {
                    // in order for output_ch() to work for k_manual we
                    // also have to override radio_in
                    rc->set_radio_in(pwm);
                }
            }
        }
    }
}

/*
  return true if a particular function is assigned to at least one RC channel
 */
bool
SRV_Channels::function_assigned(SRV_Channel::Aux_servo_function_t function)
{
    return function_mask.get(uint16_t(function));
}

/*
  set servo_out and angle_min/max, then calc_pwm and output a
  value. This is used to move a AP_Mount servo
 */
void
SRV_Channels::move_servo(SRV_Channel::Aux_servo_function_t function,
                         int16_t value, int16_t angle_min, int16_t angle_max)
{
    if (!function_assigned(function)) {
        return;
    }
    if (angle_max <= angle_min) {
        return;
    }
    float v = float(value - angle_min) / float(angle_max - angle_min);
    v = constrain_float(v, 0.0f, 1.0f);
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &ch = channels[i];
        if (ch.function.get() == function) {
            float v2 = ch.get_reversed()? (1-v) : v;
            uint16_t pwm = ch.servo_min + v2 * (ch.servo_max - ch.servo_min);
            ch.set_output_pwm(pwm);
        }
    }
}

/*
  set the default channel an auxiliary output function should be on
 */
bool SRV_Channels::set_aux_channel_default(SRV_Channel::Aux_servo_function_t function, uint8_t channel)
{
    if (!initialised) {
        update_aux_servo_function();
    }
    if (function_assigned(function)) {
        // already assigned
        return true;
    }
    if (channels[channel].function != SRV_Channel::k_none) {
        if (channels[channel].function == function) {
            return true;
        }
        hal.console->printf("Channel %u already assigned %u\n",
                            (unsigned)channel,
                            (unsigned)channels[channel].function);
        return false;
    }
    channels[channel].type_setup = false;
    channels[channel].function.set(function);
    channels[channel].aux_servo_function_setup();
    function_mask.set((uint8_t)function);
    return true;
}

// find first channel that a function is assigned to
bool SRV_Channels::find_channel(SRV_Channel::Aux_servo_function_t function, uint8_t &chan)
{
    if (!initialised) {
        update_aux_servo_function();
    }
    if (!function_assigned(function)) {
        return false;
    }
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            chan = channels[i].ch_num;
            return true;
        }
    }
    return false;
}

/*
  get a pointer to first auxillary channel for a channel function
*/
SRV_Channel *SRV_Channels::get_channel_for(SRV_Channel::Aux_servo_function_t function, int8_t default_chan)
{
    uint8_t chan;
    if (default_chan >= 0) {
        set_aux_channel_default(function, default_chan);
    }
    if (!find_channel(function, chan)) {
        return nullptr;
    }
    return &channels[chan];
}

void SRV_Channels::set_output_scaled(SRV_Channel::Aux_servo_function_t function, int16_t value)
{
    if (function < SRV_Channel::k_nr_aux_servo_functions) {
        functions[function].output_scaled = value;
        SRV_Channel::have_pwm_mask &= ~functions[function].channel_mask;
    }
}

int16_t SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t function)
{
    if (function < SRV_Channel::k_nr_aux_servo_functions) {
        return functions[function].output_scaled;
    }
    return 0;
}


// set the trim for a function channel to given pwm
void SRV_Channels::set_trim_to_pwm_for(SRV_Channel::Aux_servo_function_t function, int16_t pwm)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].servo_trim.set(pwm);
        }
    }
}

// set the trim for a function channel to min output
void SRV_Channels::set_trim_to_min_for(SRV_Channel::Aux_servo_function_t function)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].servo_trim.set(channels[i].get_reversed()?channels[i].servo_max:channels[i].servo_min);
        }
    }
}

/*
  set the default function for a channel
*/
void SRV_Channels::set_default_function(uint8_t chan, SRV_Channel::Aux_servo_function_t function)
{
    if (chan < NUM_SERVO_CHANNELS) {
        int8_t old = channels[chan].function;
        channels[chan].function.set_default((uint8_t)function);
        if (old != channels[chan].function && channels[chan].function == function) {
            function_mask.set((uint8_t)function);
        }
    }
}


void SRV_Channels::set_esc_scaling_for(SRV_Channel::Aux_servo_function_t function)
{
    uint8_t chan;
    if (find_channel(function, chan)) {
        hal.rcout->set_esc_scaling(channels[chan].get_output_min(), channels[chan].get_output_max());
    }
}

/*
  auto-adjust channel trim from an integrator value. Positive v means
  adjust trim up. Negative means decrease
 */
void SRV_Channels::adjust_trim(SRV_Channel::Aux_servo_function_t function, float v)
{
    if (is_zero(v)) {
        return;
    }
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (function != (SRV_Channel::Aux_servo_function_t)(c.function.get())) {
            continue;
        }
        float change = c.reversed?-v:v;
        uint16_t new_trim = c.servo_trim;
        float trim_scaled = float(c.servo_trim - c.servo_min) / (c.servo_max - c.servo_min);
        if (change > 0 && trim_scaled < 0.6f) {
            new_trim++;
        } else if (change < 0 && trim_scaled > 0.4f) {
            new_trim--;
        } else {
            return;
        }
        c.servo_trim.set(new_trim);

        trimmed_mask |= 1U<<i;
    }
}


// get pwm output for the first channel of the given function type.
bool SRV_Channels::get_output_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t &value)
{
    uint8_t chan;
    if (!find_channel(function, chan)) {
        return false;
    }
    channels[chan].calc_pwm(functions[function].output_scaled);
    value = channels[chan].output_pwm;
    return true;
}

// set output pwm to trim for the given function
void SRV_Channels::set_output_to_trim(SRV_Channel::Aux_servo_function_t function)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_output_pwm(channels[i].servo_trim);
        }
    }
}

// set output pwm to for first matching channel
void SRV_Channels::set_output_pwm_first(SRV_Channel::Aux_servo_function_t function, uint16_t pwm)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_output_pwm(pwm);
            break;
        }
    }
}

/*
  get the normalised output for a channel function from the pwm value
  of the first matching channel
 */
float SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t function)
{
    uint8_t chan;
    if (!find_channel(function, chan)) {
        return 0;
    }
    channels[chan].calc_pwm(functions[function].output_scaled);
    return channels[chan].get_output_norm();
}

/*
  limit slew rate for an output function to given rate in percent per
  second. This assumes output has not yet done to the hal
 */
void SRV_Channels::limit_slew_rate(SRV_Channel::Aux_servo_function_t function, float slew_rate, float dt)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &ch = channels[i];
        if (ch.function == function) {
            ch.calc_pwm(functions[function].output_scaled);
            uint16_t last_pwm = hal.rcout->read_last_sent(ch.ch_num);
            if (last_pwm == ch.output_pwm) {
                continue;
            }
            uint16_t max_change = (ch.get_output_max() - ch.get_output_min()) * slew_rate * dt * 0.01f;
            if (max_change == 0) {
                // always allow some change
                max_change = 1;
            }
            ch.output_pwm = constrain_int16(ch.output_pwm, last_pwm-max_change, last_pwm+max_change);
        }
    }
}

// call set_angle() on matching channels
void SRV_Channels::set_angle(SRV_Channel::Aux_servo_function_t function, uint16_t angle)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_angle(angle);
        }
    }    
}

// call set_range() on matching channels
void SRV_Channels::set_range(SRV_Channel::Aux_servo_function_t function, uint16_t range)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_range(range);
        }
    }
}

// constrain to output min/max for function
void SRV_Channels::constrain_pwm(SRV_Channel::Aux_servo_function_t function)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &ch = channels[i];
        if (ch.function == function) {
            ch.output_pwm = constrain_int16(ch.output_pwm, ch.servo_min, ch.servo_max);
        }
    }
}

/*
  upgrade RC* parameters into SERVO* parameters. This does the following:

  - copies MIN/MAX/TRIM values from old RC parameters into new RC* parameters and SERVO* parameters. 
  - copies RCn_FUNCTION to SERVOn_FUNCTION
  - maps old RCn_REV to SERVOn_REVERSE and RCn_REVERSE

  aux_channel_mask is a bitmask of which channels were RC_Channel_aux channels

  Note that this code is highly dependent on the parameter indexing of
  the old RC_Channel and RC_Channel_aux objects.

  If rcmap is passed in then the vehicle code also wants functions for
  the first 4 output channels to be remapped

  We return true if an upgrade has been done. This allows the caller
  to make any vehicle specific upgrades that may be needed
*/
bool SRV_Channels::upgrade_parameters(const uint8_t rc_keys[14], uint16_t aux_channel_mask, RCMapper *rcmap)
{
    // use SERVO16_FUNCTION as a marker to say that we have run the upgrade already
    if (channels[15].function.configured_in_storage()) {
        // upgrade already done
        return false;
    }
    
    // old system had 14 RC channels
    for (uint8_t i=0; i<14; i++) {
        uint8_t k = rc_keys[i];
        if (k == 0) {
            // missing parameter. Some vehicle types didn't have all parameters
            continue;
        }
        SRV_Channel &srv_chan = channels[i];
        RC_Channel &rc_chan = RC_Channels::channels[i];
        enum {
            FLAG_NONE=0,
            FLAG_IS_REVERSE=1,
            FLAG_AUX_ONLY=2
        };
        const struct mapping {
            uint8_t old_index;
            AP_Param *new_srv_param;
            AP_Param *new_rc_param;
            enum ap_var_type type;
            uint8_t flags;
        } mapping[] = {
            { 0, &srv_chan.servo_min,  &rc_chan.radio_min,  AP_PARAM_INT16, FLAG_NONE },
            { 1, &srv_chan.servo_trim, &rc_chan.radio_trim, AP_PARAM_INT16, FLAG_NONE },
            { 2, &srv_chan.servo_max,  &rc_chan.radio_max,  AP_PARAM_INT16, FLAG_NONE },
            { 3, &srv_chan.reversed,   &rc_chan.reversed,   AP_PARAM_INT8,  FLAG_IS_REVERSE },
            { 1, &srv_chan.function,   nullptr,             AP_PARAM_INT8,  FLAG_AUX_ONLY },
        };
        bool is_aux = aux_channel_mask & (1U<<i);
        
        for (uint8_t j=0; j<ARRAY_SIZE(mapping); j++) {
            const struct mapping &m = mapping[j];
            AP_Param::ConversionInfo info;
            AP_Int8 v8;
            AP_Int16 v16;
            AP_Param *v = m.type == AP_PARAM_INT16?(AP_Param*)&v16:(AP_Param*)&v8;
            bool aux_only = (m.flags & FLAG_AUX_ONLY)!=0;
            if (!is_aux && aux_only) {
                continue;
            }
            info.old_key = k;
            info.type = m.type;
            info.new_name = nullptr;

            // if this was an aux channel we need to shift by 6 bits, but not for RCn_FUNCTION
            info.old_group_element = (is_aux && !aux_only)?(m.old_index<<6):m.old_index;
            
            if (!AP_Param::find_old_parameter(&info, v)) {
                // the parameter wasn't set in the old eeprom
                continue;
            }

            if (m.flags & FLAG_IS_REVERSE) {
                // special mapping from RCn_REV to RCn_REVERSED
                v8.set(v8.get() == -1?1:0);
            }
            
            if (!m.new_srv_param->configured_in_storage()) {
                // not configured yet in new eeprom
                if (m.type == AP_PARAM_INT16) {
                    ((AP_Int16 *)m.new_srv_param)->set_and_save_ifchanged(v16.get());
                } else {
                    ((AP_Int8 *)m.new_srv_param)->set_and_save_ifchanged(v8.get());
                }
            }
            if (m.new_rc_param && !m.new_rc_param->configured_in_storage()) {
                // not configured yet in new eeprom
                if (m.type == AP_PARAM_INT16) {
                    ((AP_Int16 *)m.new_rc_param)->set_and_save_ifchanged(v16.get());
                } else {
                    ((AP_Int8 *)m.new_rc_param)->set_and_save_ifchanged(v8.get());
                }
            }
        }
    }

    if (rcmap != nullptr) {
        // we need to make the output functions from the rcmapped inputs
        const int8_t func_map[4] = { channels[0].function.get(),
                                     channels[1].function.get(),
                                     channels[2].function.get(),
                                     channels[3].function.get() };
        const uint8_t map[4] = { rcmap->roll(), rcmap->pitch(), rcmap->throttle(), rcmap->yaw() };
        for (uint8_t i=0; i<4; i++) {
            uint8_t m = uint8_t(map[i]-1);
            if (m != i && m < 4) {
                channels[m].function.set_and_save_ifchanged(func_map[i]);
            }
        }
    }

    
    // mark the upgrade as having been done
    channels[15].function.set_and_save(channels[15].function.get());

    return true;
}



/*
  Upgrade servo MIN/MAX/TRIM/REVERSE parameters for a single AP_Motors
  RC_Channel servo from previous firmwares, setting the equivalent
  parameter in the new SRV_Channels object
*/
void SRV_Channels::upgrade_motors_servo(uint8_t ap_motors_key, uint8_t ap_motors_idx, uint8_t new_channel)
{
    SRV_Channel &srv_chan = channels[new_channel];
    enum {
        FLAG_NONE=0,
        FLAG_IS_REVERSE=1
    };
    const struct mapping {
        uint8_t old_index;
        AP_Param *new_srv_param;
        enum ap_var_type type;
        uint8_t flags;
    } mapping[] = {
            { 0, &srv_chan.servo_min,  AP_PARAM_INT16, FLAG_NONE },
            { 1, &srv_chan.servo_trim, AP_PARAM_INT16, FLAG_NONE },
            { 2, &srv_chan.servo_max,  AP_PARAM_INT16, FLAG_NONE },
            { 3, &srv_chan.reversed,   AP_PARAM_INT8,  FLAG_IS_REVERSE },
    };
        
    for (uint8_t j=0; j<ARRAY_SIZE(mapping); j++) {
        const struct mapping &m = mapping[j];
        AP_Param::ConversionInfo info;
        AP_Int8 v8;
        AP_Int16 v16;
        AP_Param *v = m.type == AP_PARAM_INT16?(AP_Param*)&v16:(AP_Param*)&v8;

        info.old_key = ap_motors_key;
        info.type = m.type;
        info.new_name = nullptr;
        info.old_group_element = ap_motors_idx | (m.old_index<<6);
        
        if (!AP_Param::find_old_parameter(&info, v)) {
            // the parameter wasn't set in the old eeprom
            continue;
        }

        if (m.flags & FLAG_IS_REVERSE) {
            // special mapping from RCn_REV to RCn_REVERSED
            v8.set(v8.get() == -1?1:0);
        }

        // we save even if there is already a value in the new eeprom,
        // as that may come from the equivalent RC channel, not the
        // old motor servo channel
        if (m.type == AP_PARAM_INT16) {
            ((AP_Int16 *)m.new_srv_param)->set_and_save_ifchanged(v16.get());
        } else {
            ((AP_Int8 *)m.new_srv_param)->set_and_save_ifchanged(v8.get());
        }
    }
}


// set RC output frequency on a function output
void SRV_Channels::set_rc_frequency(SRV_Channel::Aux_servo_function_t function, uint16_t frequency_hz)
{
    uint16_t mask = 0;
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &ch = channels[i];
        if (ch.function == function) {
            mask |= (1U<<ch.ch_num);
        }
    }
    if (mask != 0) {
        hal.rcout->set_freq(mask, frequency_hz);
    }
}
